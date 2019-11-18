///////////////////////////////////////////////////////////////////////////////
// velma_ec_driver.cpp
//
// Main EtherCAT driver worker module
// Initializes CIFX card driver, shared memory buffers and exchanges data
//  with EtherCAT slaves devices cyclically within period defined in EtherCAT
//  ENI file (by default located in /opt/cifx/...)
//
///////////////////////////////////////////////////////////////////////////////

#include <cstring>
#include <csignal>

#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>

#include <pthread.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>

#include "cifx/linux/cifx.h"
#include "cifx/cifXToolkit.h"
#include "cifx/cifXErrors.h"
#include "cifx/TLR_Types.h"
#include "cifx/TLR_Results.h"
#include "cifx/rcX_Public.h"
#include "cifx/EcmIF_Public.h"

#include "shm_comm/Reader.hpp"
#include "shm_comm/Writer.hpp"
#include "shm_comm/shm_err.h"

///////////////////////////////////////////////////////////////////////////////
// Private declarations
///////////////////////////////////////////////////////////////////////////////

using namespace std::chrono_literals;

static void sync_callback(uint32_t notification, uint32_t data_len,
                          void* data, void* user);

static int32_t ecat_get_timing(CIFXHANDLE handle, ECM_IF_GET_TIMING_INFO_CNF_DATA_T *timing_data);

static int32_t ecat_config_sync(CIFXHANDLE handle);

static void interrupt(int data);

static void lock_memory(int flags);

static void set_sched_params(int policy, int priority);

static void set_affinity(unsigned int affinity);

///////////////////////////////////////////////////////////////////////////////
// Private globals
///////////////////////////////////////////////////////////////////////////////

static std::atomic<bool> stop {false};

///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

int main(int /*argc*/, char** /*argv*/)
{
    const size_t pd_data_size = 1536;
    const auto affinity = 0x2;

    printf("Installing signal handler...\n");

    signal(SIGINT, interrupt);

    printf("Opening shared memory channels...\n");
    auto ec_command = shm::Reader("EC_Command");
    auto ec_status = shm::Writer("EC_Status");

    CIFXHANDLE driver = NULL;
    CIFXHANDLE hChannel = NULL;
    uint32_t ulChannel = 0;
    uint32_t ulState = 0;
    int32_t sts;

    char ErrorStr[200];

    printf("Configuring RT properties...\n");
    lock_memory(MCL_CURRENT | MCL_FUTURE);
    set_sched_params(SCHED_FIFO, 20);

    printf("Initializing EtherCAT...\n");

    CIFX_DEVICE_INFO_T device;
    device.uio_num = 0;
    device.irq_sched_policy = SCHED_FIFO;
    device.irq_sched_priority = 40;
    device.irq_affinity = affinity;

    CIFX_LINUX_INIT_T init;
    init.poll_interval_ms = -1; // Not using COS-polling
    init.trace_level = TRACE_LEVEL_INFO;
    init.devices = &device;
    init.devices_count = 1;

    const auto ec = cifXDriverInit(&init);
    if(ec != CIFX_NO_ERROR)
    {
        TRACE_DRIVER_ERROR(ec, "[velma_ec_driver] Could not initialize driver");
        xDriverGetErrorDescription(ec, ErrorStr, sizeof(ErrorStr));
        printf("error : %X\n", ec);
        printf("%s\n", ErrorStr);
        return -1;
    }

    sts = xDriverOpen(&driver);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %X\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    sts = xChannelOpen(driver, "cifx0", ulChannel, &hChannel);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %X\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    sts = xChannelHostState(hChannel, CIFX_HOST_STATE_READY, &ulState, 100);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %d\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    sts = xChannelRegisterNotification(hChannel, CIFX_NOTIFY_COM_STATE, &sync_callback, NULL);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %d\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    std::cout << "Waiting 3 s..." << std::endl;
    std::this_thread::sleep_for(3s);

    sts = ecat_config_sync(hChannel);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error ecat_config_sync: %d\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    ECM_IF_GET_TIMING_INFO_CNF_DATA_T tdata;
    sts = ecat_get_timing(hChannel, &tdata);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error ecat_get_timing: %u\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    printf("EtherCAT bus cycle: %dns\n", tdata.ulBusCycleTimeNs);
    printf("EtherCAT frame transmit time: %dns\n", tdata.ulFrameTransmitTimeNs);

    set_affinity(affinity);

    printf("Started!\n");
    while(!stop)
    {
        void* ec_status_buffer {nullptr};
        if(const auto result = ec_status.try_buffer_get(&ec_status_buffer); result != 0)
        {
            printf("Could not get EC_Status write buffer, result: %d\n", result);
        }
        else if(const auto result = xChannelIORead(hChannel, 0, 0, pd_data_size, ec_status_buffer, 1000); result != CIFX_NO_ERROR)
        {
            xDriverGetErrorDescription(result, ErrorStr, sizeof(ErrorStr));
            printf("EtherCAT Channel IO read error: %d\n", result);
            printf("%s\n", ErrorStr);
        }
        else if(const auto result = ec_status.try_buffer_write(); result != 0)
        {
            printf("Could not write EC_Status buffer, result: %d\n", result);
        }

        void* ec_command_buffer {nullptr};
        if(const auto result = ec_command.try_buffer_get(&ec_command_buffer); result != 0)
        {
            if(result != SHM_NODATA)
            {
                printf("Could not get EC_Command read buffer, result: %d\n", result);
            }
        }
        else if(const auto result = xChannelIOWrite(hChannel, 0, 0, pd_data_size, ec_command_buffer, 1000); result != CIFX_NO_ERROR)
        {
            xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
            printf("Channel write error: %d\n", sts);
            printf("%s\n", ErrorStr);
        }
    }

    set_affinity(0x1);

    printf("Closing ec driver...");

    sts = xChannelBusState(hChannel, CIFX_BUS_STATE_OFF, &ulState, 1000);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("ChannelBusState set to OFF error : %d\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    sts = xChannelHostState(hChannel, CIFX_HOST_STATE_NOT_READY, &ulState, 100);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("ChannelHostState set to NotReady error : %d\n", sts);
        printf("%s\n", ErrorStr);
        return -1;
    }

    xChannelClose(hChannel);
    xDriverClose(driver);
}

void sync_callback(uint32_t /*notification*/, uint32_t /*data_len*/,
                   void* data, void* /*user*/)
{
	auto state = reinterpret_cast<CIFX_NOTIFY_COM_STATE_T*>(data);
    printf("EtherCAT Sync Callback, comm state : %d\n", state->ulComState);
}

int32_t ecat_get_timing(CIFXHANDLE handle, ECM_IF_GET_TIMING_INFO_CNF_DATA_T *timing_data)
{
    // Send get timing request

    ECM_IF_GET_TIMING_INFO_REQ_T conf_req;
    conf_req.tHead.ulDest = 0x00000020;
    conf_req.tHead.ulLen = 0;
    conf_req.tHead.ulId = 1;
    conf_req.tHead.ulSta = 0;
    conf_req.tHead.ulCmd = ECM_IF_CMD_GET_TIMING_INFO_REQ;
    conf_req.tHead.ulExt = 0;
    conf_req.tHead.ulRout = 0;

    const auto put_status = xChannelPutPacket(handle, (CIFX_PACKET*)&conf_req, 100);
    if(put_status != CIFX_NO_ERROR)
    {
        printf("ecat_get_timing, put packet error %d\n", put_status);
        return put_status;
    }

    // Wait for response

    ECM_IF_GET_TIMING_INFO_CNF_T config_res;
    const auto get_timeout_ms = 5000;
    const auto get_status = xChannelGetPacket(handle, sizeof(config_res), (CIFX_PACKET*)&config_res, get_timeout_ms);
    if(get_status != CIFX_NO_ERROR)
    {
        printf("ecat_get_timing, get packet error %d\n", get_status);
        return get_status;
    }

    (*timing_data) = config_res.tData;

    return config_res.tHead.ulSta;
}

int32_t ecat_config_sync(CIFXHANDLE handle)
{
    int32_t sts;

    RCX_SET_HANDSHAKE_CONFIG_REQ_T conf_req;
    conf_req.tHead.ulDest = 0x00000020;
    conf_req.tHead.ulLen = 20;
    conf_req.tHead.ulId = 1;
    conf_req.tHead.ulSta = 0;
    conf_req.tHead.ulCmd = RCX_SET_HANDSHAKE_CONFIG_REQ;
    conf_req.tHead.ulExt = 0;
    conf_req.tHead.ulRout = 0;

    conf_req.tData.bPDInHskMode = 5;
    conf_req.tData.bPDInSource = 0;
    conf_req.tData.bPDOutHskMode = 4;
    conf_req.tData.bPDOutSource = 0;
    conf_req.tData.usPDOutErrorTh = 0;
    conf_req.tData.usPDInErrorTh = 0;
    conf_req.tData.bSyncHskMode = 0;
    conf_req.tData.bSyncSource = 0;
    conf_req.tData.usSyncErrorTh = 0;

    sts = xChannelPutPacket(handle, (CIFX_PACKET*)&conf_req, 100);

    if(sts != CIFX_NO_ERROR)
    {
      std::cout << "ecat_config_sync error 1" << std::endl;
      return sts;
    }

    // wait for reply
    RCX_SET_HANDSHAKE_CONFIG_CNF_T config_res;

    sts = xChannelGetPacket(handle, sizeof(config_res), (CIFX_PACKET*)&config_res, 5000);

    if(sts != CIFX_NO_ERROR)
    {
      std::cout << "ecat_config_sync error 2" << std::endl;
      return sts;
    }

    return config_res.tHead.ulSta;
}

void interrupt(int /*data*/)
{
    stop = true;
}

void lock_memory(int flags)
{
    printf("[velma_ec_driver] Locking process memory with flags 0x%x...\n", flags);

    const auto result = mlockall(flags);
    if(result != 0)
    {
        const auto ec = errno;
        throw std::runtime_error("Could not lock process memory: "
            + std::string(strerror(ec)));
    }
}

void set_sched_params(int policy, int priority)
{
    printf("[velma_ec_driver] Setting sched policy to %d with priority %d\n", policy, priority);

    struct sched_param param;
    param.sched_priority = priority;
    const auto ec = pthread_setschedparam(pthread_self(), policy, &param);
    if(ec != 0)
    {
        throw std::runtime_error("Could not set sched params: "
            + std::string(strerror(ec)));
    }
}

void set_affinity(unsigned int affinity)
{
    cpu_set_t cs;
    CPU_ZERO(&cs);
    for(unsigned i = 0; i < 8*sizeof(affinity); i++)
    {
        if(affinity & (1 << i))
        {
            CPU_SET(i, &cs);
        }
    }

    printf("[velma_ec_driver] Setting affinity to 0x%x...\n", affinity);

    const auto thread = pthread_self();
    const auto ec = pthread_setaffinity_np(thread, sizeof(cs), &cs);
    if(ec != 0)
    {
        throw std::runtime_error("Could not set affinity: " +
            std::string(strerror(ec)));
    }
}
