// -*- mode:c++; fill-column: 100; -*-

#include <pthread.h>

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <serial/serial.h>
#include <boost/crc.hpp>

#include "openrobot_vesc_driver/vesc_interface.h"
#include "openrobot_vesc_driver/vesc_packet_factory.h"

//#define DEBUG_VIEW_PACKET	//cdi

namespace openrobot_vesc_driver
{

class VescInterface::Impl
{
public:
  Impl() :
    serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100),
            serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none)
  {}

  void* rxThread(void);

  static void* rxThreadHelper(void *context)
  {
    return ((VescInterface::Impl*)context)->rxThread();
  }

  pthread_t rx_thread_;
  bool rx_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  serial::Serial serial_;
  VescFrame::CRC send_crc_;
};

void* VescInterface::Impl::rxThread(void)
{
  Buffer buffer;
  buffer.reserve(4096);

  while (rx_thread_run_) {

    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer.empty()) {

      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end()) {

        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
            VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter) {

          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet =
            VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);
          if (packet) {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0) {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          }
          else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break; // for (iter_sof...
          }
          else {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end())
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }

    // attempt to read at least bytes_needed bytes from the serial port
    int bytes_to_read =
      std::max(bytes_needed, std::min(4096, static_cast<int>(serial_.available())));
    int bytes_read = serial_.read(buffer, bytes_to_read);
    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
      error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
    }

  }
}


VescInterface::VescInterface(const std::string& port,
                             const std::string& baudrate,
                             const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler) :
  impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty())
    connect(port, baudrate);
}

VescInterface::~VescInterface()
{
  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port, const std::string& baud)
{
  // todo - mutex?

  if (isConnected()) {
    throw SerialException("Already connected to serial port.");
  }

  // connect to serial port
  try {
    impl_->serial_.setPort(port);
    impl_->serial_.setBaudrate(std::stoi(baud));
    impl_->serial_.open();
  }
  catch (const std::exception& e) {
      std::stringstream ss;
      ss << "Failed to open the serial port to the VESC. " << e.what();
      throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->rx_thread_run_ = true;
  int result =
    pthread_create(&impl_->rx_thread_, NULL, &VescInterface::Impl::rxThreadHelper, impl_.get());
  assert(0 == result);
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected()) {
    // bring down read thread
    impl_->rx_thread_run_ = false;
    int result = pthread_join(impl_->rx_thread_, NULL);
    assert(0 == result);

    impl_->serial_.close();
  }
}

bool VescInterface::isConnected() const
{
  return impl_->serial_.isOpen();
}

void VescInterface::send(const VescPacket& packet)
{
  size_t written = impl_->serial_.write(packet.frame());

#ifdef DEBUG_VIEW_PACKET
  if(*(packet.frame().begin()+1)==5)
  printf("frame=%d %d %d %d %d %d %d %d %d %d\n ", *(packet.frame().begin()), *(packet.frame().begin()+1), *(packet.frame().begin()+2)
  		  	  	  	  	  	  , *(packet.frame().begin()+3), *(packet.frame().begin()+4), *(packet.frame().begin()+5)
  							  , *(packet.frame().begin()+6), *(packet.frame().begin()+7), *(packet.frame().begin()+8)
  							  , *(packet.frame().begin()+9));

  if(*(packet.frame().begin()+1)==7)
  printf("frame=%d %d %d %d %d %d %d %d %d %d %d %d\n ", *(packet.frame().begin()), *(packet.frame().begin()+1), *(packet.frame().begin()+2)
		  	  	  	  	  	  , *(packet.frame().begin()+3), *(packet.frame().begin()+4), *(packet.frame().begin()+5)
							  , *(packet.frame().begin()+6), *(packet.frame().begin()+7), *(packet.frame().begin()+8)
							  , *(packet.frame().begin()+9), *(packet.frame().begin()+10), *(packet.frame().begin()+11));
#endif

  if (written != packet.frame().size()) {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.frame().size() << ".";
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

//cdi
void VescInterface::getCustomApp()
{
  send(VescPacketRequestCustomApp());
}

//cdi
void VescInterface::setCustomApp(const openrobot_vesc_msgs::VescSetCustomApp::ConstPtr& custom_set_msg)
{
  send(VescPacketSetCustomAppData(custom_set_msg));
}

//cdi
void VescInterface::setAlive()
{
  send(VescPacketSetALIVE());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

//cdi
void VescInterface::setDutyCycle(double duty_cycle, int canf_flag, int can_id)
{
  send(VescPacketSetDuty(duty_cycle, canf_flag, can_id));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

//cdi
void VescInterface::setCurrent(double current, int canf_flag, int can_id)
{
  send(VescPacketSetCurrent(current, canf_flag, can_id));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

//cdi
void VescInterface::setBrake(double brake, int canf_flag, int can_id)
{
  send(VescPacketSetCurrentBrake(brake, canf_flag, can_id));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetRPM(speed));
}

//cdi
void VescInterface::setSpeed(double speed, int canf_flag, int can_id)
{
  send(VescPacketSetRPM(speed, canf_flag, can_id));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

//cdi
void VescInterface::setPosition(double position, int canf_flag, int can_id)
{
  send(VescPacketSetPos(position, canf_flag, can_id));
}

void VescInterface::setServo(double servo)
{
  send(VescPacketSetServoPos(servo));
}

//cdi
void VescInterface::setServo(double servo, int canf_flag, int can_id)
{
  send(VescPacketSetServoPos(servo, canf_flag, can_id));
}

} // namespace openrobot_vesc_driver
