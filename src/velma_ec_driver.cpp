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

#include "shm_comm/shm_channel.h"

///////////////////////////////////////////////////////////////////////////////
// Private declarations
///////////////////////////////////////////////////////////////////////////////

using namespace std::chrono_literals;

static void sync_callback(uint32_t notification, uint32_t data_len,
                          void* data, void* user);

static int32_t ecat_get_timing(CIFXHANDLE handle, ECM_IF_GET_TIMING_INFO_CNF_DATA_T *timing_data);

static int32_t ecat_config_sync(CIFXHANDLE handle);

static void interrupt(int data);

static shm_reader_t* open_shm_reader(const char* name, size_t buffer_size);

static shm_writer_t* open_shm_writer(const char* name, size_t buffer_size);

static void lock_memory(int flags);

static void set_sched_params(int policy, int priority);

static void set_affinity(unsigned int affinity);

///////////////////////////////////////////////////////////////////////////////
// Private globals
///////////////////////////////////////////////////////////////////////////////

static bool stop = false;

///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const size_t pd_data_size = 1536;

    printf("Installing signal handler...\n");

    signal(SIGINT, interrupt);

    printf("Opening SHM channels...\n");

    auto shm_reader = open_shm_reader("EC_Command", pd_data_size);
    auto shm_writer = open_shm_writer("EC_Status", pd_data_size);

    //
    // EC stuff
    //

    CIFXHANDLE driver = NULL;
    CIFXHANDLE hChannel = NULL;
    uint32_t ulChannel = 0;
    uint32_t ulState = 0;
    int32_t sts;

    char ErrorStr[200];

    printf("Configuring RT properties...\n");

    lock_memory(MCL_CURRENT | MCL_FUTURE);
    set_sched_params(SCHED_FIFO, 20);
    set_affinity(0x2);

    printf("Initializing EtherCAT...\n");

    CIFX_DEVICE_INFO_T device;
    device.uio_num = 0;
    device.irq_sched_policy = SCHED_FIFO;
    device.irq_sched_priority = 40;

    CIFX_LINUX_INIT_T init;
    init.poll_interval_ms = 1000;
    init.poll_sched_policy = SCHED_FIFO;
    init.poll_sched_priority = 40;
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
            shm_release_reader(shm_reader);
            shm_release_writer(shm_writer);
        return -1;
    }

    sts = xDriverOpen(&driver);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %X\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
        return -1;
    }

    sts = xChannelOpen(driver, "cifx0", ulChannel, &hChannel);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %X\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
        return -1;
    }

    sts = xChannelHostState(hChannel, CIFX_HOST_STATE_READY, &ulState, 100);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %d\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
        return -1;
    }

    sts = xChannelRegisterNotification(hChannel, CIFX_NOTIFY_COM_STATE, &sync_callback, NULL);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %d\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
        return -1;
    }

    std::cout << "Waiting 5 s..." << std::endl;
    std::this_thread::sleep_for(5s);

    sts = ecat_config_sync(hChannel);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("error ecat_config_sync: %d\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
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

    void *shm_writerbuf = NULL;
    shm_writer_buffer_get(shm_writer, &shm_writerbuf);
    if(shm_writerbuf == NULL)
    {
        stop = true;
        std::cout << "ERROR: got NULL writer shm buffer" << std::endl;
    }

    printf("Starting...\n");

    void *shm_readerbuf = NULL;
    bool first_ec_error = true;
    bool first_shm_error = true;
    int loops = 0;
    while (!stop)
    {
        const auto read_status = xChannelIORead(hChannel, 0, 0, pd_data_size, shm_writerbuf, 1000);
        if(read_status != CIFX_NO_ERROR)
        {
            if(first_ec_error)
            {
                xDriverGetErrorDescription(read_status, ErrorStr, sizeof(ErrorStr));
                printf("Channel read error: %d\n", read_status);
                printf("%s\n", ErrorStr);
                first_ec_error = false;
            }

            continue;
        }

        first_ec_error = true;
        const auto buffer_get_status = shm_reader_buffer_get(shm_reader, &shm_readerbuf);

        shm_writer_buffer_write(shm_writer);
        shm_writer_buffer_get(shm_writer, &shm_writerbuf);

        if(buffer_get_status == 0 && shm_readerbuf != NULL)
        {
            first_shm_error = true;
            sts = xChannelIOWrite(hChannel, 0, 0, pd_data_size, shm_readerbuf, 1000);
            if(sts != CIFX_NO_ERROR)
            {
                xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
                printf("Channel write error: %d\n", sts);
                printf("%s\n", ErrorStr);
            }
        }
        else if(first_shm_error)
        {
            printf("SHM read error : %d\n", buffer_get_status);
            first_shm_error = false;
        }

        ++loops;
    }

    printf("Closing ec driver...");

    sts = xChannelBusState(hChannel, CIFX_BUS_STATE_OFF, &ulState, 1000);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("ChannelBusState set to OFF error : %d\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
        return -1;
    }

    sts = xChannelHostState(hChannel, CIFX_HOST_STATE_NOT_READY, &ulState, 100);
    if(sts != CIFX_NO_ERROR)
    {
        xDriverGetErrorDescription(sts, ErrorStr, sizeof(ErrorStr));
        printf("ChannelHostState set to NotReady error : %d\n", sts);
        printf("%s\n", ErrorStr);
        shm_release_reader(shm_reader);
        shm_release_writer(shm_writer);
        return -1;
    }

    shm_release_reader(shm_reader);
    shm_release_writer(shm_writer);

    xChannelClose(hChannel);
    xDriverClose(driver);
}

void sync_callback(uint32_t /*notification*/, uint32_t /*data_len*/,
                   void* data, void* /*user*/)
{
	auto state = reinterpret_cast<CIFX_NOTIFY_COM_STATE_T*>(data);
    printf("Sync Callback, comm state : %d\n", state->ulComState);
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

static shm_reader_t* open_shm_reader(const char* name, size_t buffer_size)
{
    bool create_channel = false;

    shm_reader_t* shm_reader;
    auto result = shm_connect_reader(name, &shm_reader);
    if(result == SHM_INVAL)
    {
        std::cout << "ERROR: shm_connect_reader: invalid parameters" << std::endl;
        return nullptr;
    }
    else if(result == SHM_FATAL)
    {
        std::cout << "ERROR: shm_connect_reader: memory error" << std::endl;
        return nullptr;
    }
    else if(result == SHM_NO_CHANNEL)
    {
        std::cout << "WARNING: shm_connect_reader: could not open shm object, trying to initialize the channel..." << std::endl;
        create_channel = true;
    }
    else if(result == SHM_CHANNEL_INCONSISTENT)
    {
        std::cout << "WARNING: shm_connect_reader: shm channel is inconsistent, trying to initialize the channel..." << std::endl;
        create_channel = true;
    }
    else if(result == SHM_ERR_INIT)
    {
        std::cout << "ERROR: shm_connect_reader: could not initialize channel" << std::endl;
        return nullptr;
    }
    else if(result == SHM_ERR_CREATE)
    {
        std::cout << "WARNING: shm_connect_reader: could not create reader" << std::endl;
        create_channel = true;
    }

    if(!create_channel)
    {
        void *pbuf = NULL;
        result = shm_reader_buffer_get(shm_reader, &pbuf);
        if(result < 0)
        {
            std::cout << "WARNING: shm_reader_buffer_get: error: " << result << std::endl;
            create_channel = true;
        }
    }

    if(create_channel)
    {
        result = shm_create_channel(name, buffer_size, 1, true);
        if(result != 0)
        {
            std::cout << "ERROR: create_shm_object: error: " << result << "   errno: " << errno << std::endl;
            return nullptr;
        }

        result = shm_connect_reader(name, &shm_reader);
        if(result != 0)
        {
            std::cout << "ERROR: shm_connect_reader: error: " << result << std::endl;
            return nullptr;
        }
    }

    if(stop)
    {
        shm_release_reader(shm_reader);
        return nullptr;
    }

    return shm_reader;
}

shm_writer_t* open_shm_writer(const char* name, size_t buffer_size)
{
    bool create_channel = false;

    shm_writer_t* shm_writer;
    auto result = shm_connect_writer(name, &shm_writer);
    if(result == SHM_INVAL)
    {
        printf("ERROR: shm_connect_writer(%s): invalid parameters\n", name);
        return nullptr;
    }
    else if(result == SHM_FATAL)
    {
        printf("ERROR: shm_connect_writer(%s): memory error\n", name);
        return nullptr;
    }
    else if(result == SHM_NO_CHANNEL)
    {
        printf("WARNING: shm_connect_writer(%s): could not open shm object, trying to initialize the channel...\n", name);
        create_channel = true;
    }
    else if(result == SHM_CHANNEL_INCONSISTENT)
    {
        printf("WARNING: shm_connect_writer(%s): shm channel is inconsistent, trying to initialize the channel...\n", name);
        create_channel = true;
    }
    else if(result == SHM_ERR_INIT)
    {
        printf("ERROR: shm_connect_writer(%s): could not initialize channel\n", name);
        return nullptr;
    }
    else if(result == SHM_ERR_CREATE)
    {
        printf("ERROR: shm_connect_writer(%s): could not create reader\n", name);
        return nullptr;
    }

    if(create_channel)
    {
        result = shm_create_channel(name, buffer_size, 1, true);
        if(result != 0)
        {
            printf("ERROR: shm_create_channel(%s): error: %d errno: %d\n", name, result, errno);
            return nullptr;
        }
        result = shm_connect_writer(name, &shm_writer);
        if(result != 0)
        {
            printf("ERROR: shm_connect_writer(%s): error: %d\n", name, result);
            return nullptr;
        }
    }

    if(stop)
    {
        shm_release_writer(shm_writer);
        return nullptr;
    }

    return shm_writer;
}

void lock_memory(int flags)
{
    mlockall(flags);
}

void set_sched_params(int policy, int priority)
{
    struct sched_param param;
    param.sched_priority = priority;
    pthread_setschedparam(pthread_self(), policy, &param);
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

    const auto result = pthread_setaffinity_np(pthread_self(), sizeof(cs), &cs);
    if(result != 0)
    {
        std::cout << "ERROR: pthread_setaffinity_np: " << result << std::endl;
    }
}
