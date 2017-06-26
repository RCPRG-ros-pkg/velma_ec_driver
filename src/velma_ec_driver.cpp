extern "C" {
#include <cifx_driver/cifXLinux.h>

#include <cifx_driver/TLR_Types.h>
#include <cifx_driver/rcX_Public.h>
#include <cifx_driver/ecatm4/EcmIF_Public.h>
}

#include "shm_comm/shm_channel.h"

#include <string>
#include <iostream>
#include <fstream>

#include <string.h>
#include <stdio.h>
#include <sys/mman.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>

extern "C" {
void sync_callback( uint32_t ulNotification,
                    uint32_t ulDataLen,
                    void* pvData,
                    void* pvUser)
{
	CIFX_NOTIFY_COM_STATE_T *state = reinterpret_cast<CIFX_NOTIFY_COM_STATE_T* >(pvData);

  printf("comm state : %d\n", state->ulComState);
}
}

int32_t ecat_get_timing(CIFXHANDLE handle, ECM_IF_GET_TIMING_INFO_CNF_DATA_T *timing_data)
{
    char error_str[200];
    int32_t sts;

    ECM_IF_GET_TIMING_INFO_REQ_T conf_req;
    conf_req.tHead.ulDest = 0x00000020;
    conf_req.tHead.ulLen = 0;
    conf_req.tHead.ulId = 1;
    conf_req.tHead.ulSta = 0;
    conf_req.tHead.ulCmd = ECM_IF_CMD_GET_TIMING_INFO_REQ;
    conf_req.tHead.ulExt = 0;
    conf_req.tHead.ulRout = 0;

    sts = xChannelPutPacket(handle, (CIFX_PACKET*)&conf_req, 100);

    if ( sts != CIFX_NO_ERROR) {
      std::cout << "ecat_get_timing error 1" << std::endl;
      return sts;
    }

    // wait for reply
    ECM_IF_GET_TIMING_INFO_CNF_T config_res;

    sts = xChannelGetPacket(handle, sizeof(config_res), (CIFX_PACKET*)&config_res, 5000);

    if ( sts != CIFX_NO_ERROR) {
      std::cout << "ecat_get_timing error 2" << std::endl;
      return sts;
    }

    (*timing_data) = config_res.tData;

    return config_res.tHead.ulSta;
}

int32_t ecat_config_sync(CIFXHANDLE handle)
{
    char error_str[200];
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

    if ( sts != CIFX_NO_ERROR) {
      std::cout << "ecat_config_sync error 1" << std::endl;
      return sts;
    }

    // wait for reply
    RCX_SET_HANDSHAKE_CONFIG_CNF_T config_res;

    sts = xChannelGetPacket(handle, sizeof(config_res), (CIFX_PACKET*)&config_res, 5000);

    if ( sts != CIFX_NO_ERROR) {
      std::cout << "ecat_config_sync error 2" << std::endl;
      return sts;
    }

    return config_res.tHead.ulSta;
}

static bool stop = false;

void interrupt(int data) {
    stop = true;
}

int main(int argc, char *argv[]) 
{
    const size_t pd_data_size = 1536;

    if (argc != 2) {
        std::cout << "you must provide path to eni.xml" << std::endl;
        return 0;
    }
    std::string eni_file_path(argv[1]);

    std::ifstream file (eni_file_path, std::ios::in|std::ios::binary|std::ios::ate);
    char *eni_file;
    size_t eni_file_size;
    if (file.is_open()) {
        std::streampos size;
        size = file.tellg();
        eni_file_size = size;
        eni_file = new char [size];
        file.seekg (0, std::ios::beg);
        file.read (eni_file, size);
        file.close();
    }
    else {
        std::cout << "ERROR: could not open file: '" << eni_file_path << "'" << std::endl;
        return 0;
    }

    signal(SIGINT, interrupt);
    //
    // open reader shm channel
    //
    bool create_channel = false;
    std::string reader_shm_name("EC_Command");
    shm_reader_t* re_;
    int result = shm_connect_reader(reader_shm_name.c_str(), &re_);
    if (result == SHM_INVAL) {
        std::cout << "ERROR: shm_connect_reader: invalid parameters" << std::endl;
        return false;
    }
    else if (result == SHM_FATAL) {
        std::cout << "ERROR: shm_connect_reader: memory error" << std::endl;
        return false;
    }
    else if (result == SHM_NO_CHANNEL) {
        std::cout << "WARNING: shm_connect_reader: could not open shm object, trying to initialize the channel..." << std::endl;
        create_channel = true;
    }
    else if (result == SHM_CHANNEL_INCONSISTENT) {
        std::cout << "WARNING: shm_connect_reader: shm channel is inconsistent, trying to initialize the channel..." << std::endl;
        create_channel = true;
    }
    else if (result == SHM_ERR_INIT) {
        std::cout << "ERROR: shm_connect_reader: could not initialize channel" << std::endl;
        return false;
    }
    else if (result == SHM_ERR_CREATE) {
        std::cout << "WARNING: shm_connect_reader: could not create reader" << std::endl;
        create_channel = true;
    }

    if (!create_channel) {
        void *pbuf = NULL;
        result = shm_reader_buffer_get(re_, &pbuf);
        if (result < 0) {
            std::cout << "WARNING: shm_reader_buffer_get: error: " << result << std::endl;
            create_channel = true;
        }
    }

    if (create_channel) {
        result = shm_create_channel(reader_shm_name.c_str(), pd_data_size, 1, true);
        if (result != 0) {
            std::cout << "ERROR: create_shm_object: error: " << result << "   errno: " << errno << std::endl;
            return false;
        }

        result = shm_connect_reader(reader_shm_name.c_str(), &re_);
        if (result != 0) {
            std::cout << "ERROR: shm_connect_reader: error: " << result << std::endl;
            return false;
        }
    }

    if (stop) {
        shm_release_reader(re_);
        return 0;
    }

    //
    // open writer shm channel
    //
    create_channel = false;
    const size_t writer_data_size = 1536;
    std::string writer_shm_name("EC_Status");
    shm_writer_t* wr_;
    result = shm_connect_writer(writer_shm_name.c_str(), &wr_);
    if (result == SHM_INVAL) {
        std::cout << "ERROR: shm_connect_writer(hi_st): invalid parameters" << std::endl;
        return false;
    }
    else if (result == SHM_FATAL) {
        std::cout << "ERROR: shm_connect_writer(hi_st): memory error" << std::endl;
        return false;
    }
    else if (result == SHM_NO_CHANNEL) {
        std::cout << "WARNING: shm_connect_writer(hi_st): could not open shm object, trying to initialize the channel..." << std::endl;
        create_channel = true;
    }
    else if (result == SHM_CHANNEL_INCONSISTENT) {
        std::cout << "WARNING: shm_connect_writer(hi_st): shm channel is inconsistent, trying to initialize the channel..." << std::endl;
        create_channel = true;
    }
    else if (result == SHM_ERR_INIT) {
        std::cout << "ERROR: shm_connect_writer(hi_st): could not initialize channel" << std::endl;
        return false;
    }
    else if (result == SHM_ERR_CREATE) {
        std::cout << "ERROR: shm_connect_writer(hi_st): could not create reader" << std::endl;
        return false;
    }
    if (create_channel) {
        result = shm_create_channel(writer_shm_name.c_str(), pd_data_size, 1, true);
        if (result != 0) {
            std::cout << "ERROR: create_shm_object(hi_st): error: " << result << "   errno: " << errno << std::endl;
            return false;
        }
        result = shm_connect_writer(writer_shm_name.c_str(), &wr_);
        if (result != 0) {
            std::cout << "ERROR: shm_connect_writer(hi_st): error: " << result << std::endl;
            return false;
        }
    }

    if (stop) {
        shm_release_reader(re_);
        shm_release_writer(wr_);
        return 0;
    }

    //
    // EC stuff
    //

    CIFXHANDLE driver = NULL;
    CIFXHANDLE  hChannel = NULL;
    char szBoardName[200];
    uint32_t ulChannel = 0;
    uint32_t  ulState = 0;
    int32_t sts;
    struct CIFX_LINUX_INIT init;

    strcpy(szBoardName, "cifx0");

    char ErrorStr[200];

    mlockall(MCL_CURRENT | MCL_FUTURE );

    struct sched_param param;
    param.sched_priority = 10;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    memset( &init, 0, sizeof(init));
    init.init_options  = CIFX_DRIVER_INIT_AUTOSCAN;
    init.trace_level = 255;
    init.int_prio = 20;

    sts = cifXDriverInit( &init);
    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = xDriverOpen( &driver);
    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = xChannelOpen(driver, szBoardName, ulChannel, &hChannel);
    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = xChannelHostState(hChannel, CIFX_HOST_STATE_READY, &ulState, 100);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    char destFileName[200];
    strcpy(destFileName, "ethercat.xml");
    sts = xChannelDownload(hChannel, DOWNLOAD_MODE_CONFIG, destFileName, reinterpret_cast<uint8_t* >(eni_file), eni_file_size, NULL, NULL, NULL);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("xChannelDownload error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = xChannelReset(hChannel, CIFX_CHANNELINIT, 1000);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("xChannelDownload error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = xChannelRegisterNotification(hChannel, CIFX_NOTIFY_COM_STATE, &sync_callback, NULL);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    std::cout << "waiting 5 s..." << std::endl;
    sleep(5);

    ECM_IF_GET_TIMING_INFO_CNF_DATA_T tdata;

    sts = ecat_config_sync(hChannel);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error ecat_config_sync: %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = ecat_get_timing(hChannel, &tdata);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error ecat_get_timing: %u\n", sts);
      printf("%s\n", ErrorStr);
      return -1;
    }

    printf("bus cycle: %d\n", tdata.ulBusCycleTimeNs);
    printf("frame transmit: %d\n", tdata.ulFrameTransmitTimeNs);

//    uint8_t pd_data[pd_data_size];
    void *wr_buf = NULL;
    shm_writer_buffer_get(wr_, &wr_buf);
    if (wr_buf == NULL) {
        stop = true;
        std::cout << "ERROR: got NULL writer shm buffer" << std::endl;
    }

    void *re_buf = NULL;

    int loops = 0;
    int olddata_counter = 2;
    while (!stop)
    {
      int read_status = shm_reader_buffer_get(re_, &re_buf);

      sts = xChannelIORead(hChannel, 0, 0, pd_data_size, wr_buf, 1000);
      //uint32_t time = OS_GetMilliSecCounter();
      //printf("data read [time %d]\n", time);
      if ( sts != CIFX_NO_ERROR) {
        xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %d\n", sts);
        printf("%s\n", ErrorStr);
      }
      else {
        shm_writer_buffer_write(wr_);
        shm_writer_buffer_get(wr_, &wr_buf);
      }

      if (read_status == 1 && olddata_counter < 2 && re_buf != NULL) {
        sts = xChannelIOWrite(hChannel, 0, 0, pd_data_size, re_buf, 1000);
        ++olddata_counter;
      }
      else if (read_status == 0 && re_buf != NULL) {
          sts = xChannelIOWrite(hChannel, 0, 0, pd_data_size, re_buf, 1000);
          olddata_counter = 0;
      }
      else {
          printf("error: read_status: %d (%d)\n", read_status, loops);
          sts = xChannelIOWrite(hChannel, 0, 0, pd_data_size, wr_buf, 1000);
      }
      //printf("data write\n");
      if ( sts != CIFX_NO_ERROR) {
        xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
        printf("error : %d\n", sts);
        printf("%s\n", ErrorStr);
      }
      usleep(1000);
      ++loops;
    }

    std::cout << "closing ec driver" << std::endl;

    sts = xChannelBusState(hChannel, CIFX_BUS_STATE_OFF, &ulState, 1000);
    
    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    sts = xChannelHostState(hChannel, CIFX_HOST_STATE_NOT_READY, &ulState, 100);

    if ( sts != CIFX_NO_ERROR) {
      xDriverGetErrorDescription( sts, ErrorStr, sizeof(ErrorStr));
      printf("error : %d\n", sts);
      printf("%s\n", ErrorStr);
      shm_release_reader(re_);
      shm_release_writer(wr_);
      return -1;
    }

    shm_release_reader(re_);
    shm_release_writer(wr_);

    xChannelClose(hChannel);

    xDriverClose(driver);
}
