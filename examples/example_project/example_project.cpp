/**
 *  @example example_project.cpp
 */

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <libserial/SerialStream.h>

#include <iomanip>

#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <deque>

#define RX_ANTENNA_SPACE 2.35e-3
#define TX_ANTENNA_SPACE 5.54e-3
#define FFT_SAMPLE 151
#define CHIRP 1
#define TX_NUM 1
#define RX_NUM 4
#define IQ_NUM 2
#define HEADER_SIZE 4
#define END_SIZE 4

const int FRAME_LENGTH=(FFT_SAMPLE*CHIRP*2*IQ_NUM+IQ_NUM+HEADER_SIZE+END_SIZE)*TX_NUM*RX_NUM;

void* thread_read_fun(void *arg);
void* thread_process_fun(void *arg);

LibSerial::SerialStream serial_stream ;
std::deque<uint8_t> queue_usb;

const int frame_length=FRAME_LENGTH;

pthread_mutex_t queue_mtx=PTHREAD_MUTEX_INITIALIZER;

int read_cnt=0;

int main()
{
    // using LibSerial::SerialPort ;

    // You can instantiate a Serial Port or a Serial Stream object, whichever you'd prefer to work with.
    // For this example, we will demonstrate by using both types of objects.
    // SerialPort serial_port ;
    

    // Open hardware serial ports using the Open() method.
    // serial_port.Open( "/dev/ttyUSB0" ) ;
    using namespace LibSerial;
    {
    // serial_port.SetBaudRate( BaudRate::BAUD_115200 ) ;
        serial_stream.SetBaudRate( BaudRate::BAUD_115200 ) ;
        serial_stream.SetParity(Parity::PARITY_NONE);
        serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial_stream.SetStopBits(StopBits::STOP_BITS_1);
        serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    }
    serial_stream.Open( "/dev/ttyACM0",(std::ios_base::in|std::ios_base::out) ) ;
    serial_stream.FlushInputBuffer();
    serial_stream.FlushIOBuffers();
    // Set the baud rates.


    pthread_t thread_read;
    pthread_t thread_process;

    pthread_create(&thread_read,NULL,thread_read_fun,NULL);
    pthread_create(&thread_process,NULL,thread_process_fun,NULL);
    // Put your main loop here
    for(;;);
    {

    }
    return 0;
}


void *thread_read_fun(void *arg) {
    if(not serial_stream.IsOpen())
    {
        std::cout<<"Serial is not open\n"<<std::endl;
        // return NULL;
        throw std::runtime_error("serial not open");
    }
    else
    {
        std::cout<<"read_fun is created\n"<<std::endl;
        serial_stream<<"sensorStart\n";
    }

    while(true)
    {
        int byte_num=serial_stream.GetNumberOfBytesAvailable();
        for(int i=0;i<byte_num;i++)
        {
            uint8_t read_byte;

            try
            {
                serial_stream>>read_byte;
                pthread_mutex_lock(&queue_mtx);
                queue_usb.push_back(read_byte);
                pthread_mutex_unlock(&queue_mtx);
                // std::cout<<std::hex<<std::setw(2)<<int32_t(read_byte)<<" ";
                read_cnt++;
            }
            catch(const LibSerial::ReadTimeout&)
            {
                std::cerr << "The Read() call has timed out." << std::endl ;
            }
        }

        if(queue_usb.size()>50)
        {
            while(!queue_usb.empty() && queue_usb.front() != 0x56)
            {
                pthread_mutex_lock(&queue_mtx);
                queue_usb.pop_front();
                pthread_mutex_unlock(&queue_mtx);
                // std::cout.setf(std::ios::hex);
                // std::cout.unsetf(std::ios::hex);
            }               
        }

        if(queue_usb.size()>4 && (queue_usb[1] != 0x41))
        {
            pthread_mutex_lock(&queue_mtx);
            queue_usb.clear();       
            pthread_mutex_unlock(&queue_mtx);            
        }
        else
        {
            int frame_num=(queue_usb[2]&0x0F);
            if(queue_usb.size()>FRAME_LENGTH)
            {
                // for(auto i=queue_usb.begin();i<queue_usb.end();i++)
                // {
                //     std::cout<<std::to_string(*i)<<" ";
                // }
                // std::cout<<std::endl;
                std::cout<<frame_num<<std::endl;
                queue_usb.clear();
            }
        }
    }
    std::cout << "End fo thread_read\n";
    serial_stream.Close();
    return NULL;
}


void *thread_process_fun(void *arg) {
    while (true)
    {
        // std::cout<<"current read:"<<read_cnt<<std::endl;
        // std::cout<<"queue size:"<<queue_usb.size()<<std::endl;
        // if(queue_usb.size()>3)
        // {
        //     std::cout<<std::to_string(queue_usb[0])<<"|"<<std::to_string(queue_usb[1])<<"|"<<std::to_string(queue_usb[2])<<std::endl;
        // }
        // usleep(1000*1000);
        
    }
    

    return NULL;
}
