// -*- C++ -*-
/*!
 * @file  IMURazor9DOF.cpp
 * @brief 9DOF Razor IMU (sparfun) RT component
 * @date $Date$
 *
 * @copyright Ogata Laboratory, Waseda University, 2014
 */

#include "IMURazor9DOF.h"
#include "TimeSpec.h"
#include "Timer.h"
#include "Thread.h"

// Module specification
// <rtc-template block="module_spec">
static const char* imurazor9dof_spec[] =
  {
    "implementation_id", "IMURazor9DOF",
    "type_name",         "IMURazor9DOF",
    "description",       "9DOF Razor IMU (sparfun) RT component",
    "version",           "1.0.0",
    "vendor",            "Ogata Lab",
    "category",          "Experimental",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",
    "conf.default.port", "\\\\.\\COM20",
    "conf.default.baudrate", "57600",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.port", "text",
    "conf.__widget__.baudrate", "text",
    // Constraints

	"exec_cxt.periodic.rate", "100000",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
IMURazor9DOF::IMURazor9DOF(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_accelOut("accel", m_accel),
    m_angularVelOut("angularVel", m_angularVel),
    m_magnetOut("magnet", m_magnet)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
IMURazor9DOF::~IMURazor9DOF()
{
}



RTC::ReturnCode_t IMURazor9DOF::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("accel", m_accelOut);
  addOutPort("angularVel", m_angularVelOut);
  addOutPort("magnet", m_magnetOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("port", m_port, "COM2");
  bindParameter("baudrate", m_baudrate, "57600");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IMURazor9DOF::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IMURazor9DOF::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IMURazor9DOF::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t IMURazor9DOF::onActivated(RTC::UniqueId ec_id)
{
	std::cout << "[RTC.IMURazor9DOF] onActivated" << std::endl;
	std::cout << "[RTC.IMURazor9DOF] Opening " << m_port << std::endl;
	try {
		m_pSerialPort = new ssr::SerialPort(m_port.c_str(), m_baudrate);
	} catch (ssr::ComException& ex) {
		std::cout << "[RTC.IMURazor9DOF] Opening " << m_port << " failed." << std::endl;
		std::cout << "[RTC.IMURazor9DOF] Exception: " << ex.what() << std::endl;
		m_pSerialPort = NULL;
		return RTC::RTC_ERROR;
	}

	m_pSerialPort->FlushRxBuffer();

	ssr::Timer timer;
	timer.tick();
	
	std::cout << "[RTC.IMURazor9DOF] Reading Line..." << std::endl;
	char buffer[256];
	ssr::TimeSpec timeout(3, 0);
	int ret = m_pSerialPort->ReadLineWithTimeout(buffer, 256, timeout, "\x0A\x0D"); // CRLF Throw away
	ret = m_pSerialPort->ReadLineWithTimeout(buffer, 256, timeout, "\x0A\x0D"); // CRLF
	if(ret > 0) {
		std::cout << "[RTC.IMURazor9DOF] Read Line is :" << buffer << std::endl;
	}
	while (true) { // Timeout
		if(buffer[0] == '$') {
			break;
		}
		std::cout << "[RTC.IMURazor9DOF] Invalid Raw Data Line. Sending Ctrl+Z (0x1A) Data." << std::endl;
		char buf = 0x1A; // Ctrl + Z
		m_pSerialPort->Write(&buf, 1);
		std::cout << "[RTC.IMURazor9DOF] Sleeping 1sec and Flush Received Data" << std::endl;
		ssr::Thread::Sleep(1000);
		m_pSerialPort->FlushRxBuffer();

		std::cout << "[RTC.IMURazor9DOF] Reading Line..." << std::endl;
		m_pSerialPort->ReadLineWithTimeout(buffer, 256, timeout, "\x0A\x0D");
		ret = m_pSerialPort->ReadLineWithTimeout(buffer, 256, timeout, "\x0A\x0D"); // CRLF
		if(ret < 0) {
			std::cout << "[RTC.IMURazor9DOF] Timeout" << std::endl;
			return RTC::RTC_ERROR;
		}
	}
	std::cout << "[RTC.IMURazor9DOF] Successfully Initialized." << std::endl;
	m_timer.tick();

	m_pThread = new WorkerThread(this);
	m_pThread->Start();
	return RTC::RTC_OK;
}


RTC::ReturnCode_t IMURazor9DOF::onDeactivated(RTC::UniqueId ec_id)
{
	m_pThread->stop();
	delete m_pSerialPort;
  return RTC::RTC_OK;
}




RTC::ReturnCode_t IMURazor9DOF::onExecute(RTC::UniqueId ec_id)
{
	return RTC::RTC_OK;
}

RTC::ReturnCode_t IMURazor9DOF::onProcess()
{
	ssr::TimeSpec currentTime;
	m_timer.tack(&currentTime);
	ssr::TimeSpec timeout(3, 0);
	char buffer[256];
	//int ret = m_pSerialPort->ReadLineWithTimeout(buffer, 256, timeout, "\x0A\x0D"); // CRLF
	int ret = m_pSerialPort->ReadLine(buffer, 256, "\x0A\x0D"); // CRLF
	if (ret < 0) {
		std::cout << "[RTC.IMURazor9DOF] In OnExecute: Timeout" << std::endl;
		return RTC::RTC_ERROR;
	}

	// Error Check
	if (buffer[0] != '$' || buffer[strlen(buffer) - 1] != '#') {
		std::cout << "[RTC.IMURazor9DOF] In OnExecute: Invalid Header" << std::endl;
		return RTC::RTC_OK;
	}
	/**
	int num_comma = 0;
	for(int i = 0;i < strlen(buffer);i++) {
	if(buffer[i] == ',') {
	num_comma++;
	}
	}
	if(num_comma != 8) {
	std::cout << "Invalid Data (Invalid Number of Element" << std::endl;
	return RTC::RTC_OK;
	}
	*/


	int rax, ray, raz, rwx, rwy, rwz, rmx, rmy, rmz;
	int res = sscanf(buffer, "$%d, %d, %d, %d, %d, %d, %d, %d, %d#", &rax, &ray, &raz, &rwx, &rwy, &rwz, &rmx, &rmy, &rmz);
	if (res == EOF) {
		std::cout << "[RTC.IMURazor9DOF] In OnExecute: Invalid Format" << std::endl;
		return RTC::RTC_OK;
	}

	double ax, ay, az, wx, wy, wz, mx, my, mz;

#define G 9.80665
#define PI 3.1415926

	// 2G mode
	m_accel.data.ax = ax = rax / 255.0 * G; // m/sec^2
	m_accel.data.ay = ay = ray / 255.0 * G;
	m_accel.data.az = az = raz / 255.0 * G;
	::setTimestamp<RTC::TimedAcceleration3D>(m_accel);
	m_accelOut.write();

	m_angularVel.data.avx = wx = rwx / ((double)(0x7FFF)) * 2000.0 / 180 * PI; // rad/sec
	m_angularVel.data.avy = wy = rwy / ((double)(0x7FFF)) * 2000.0 / 180 * PI;
	m_angularVel.data.avz = wz = rwz / ((double)(0x7FFF)) * 2000.0 / 180 * PI;
	::setTimestamp<RTC::TimedAngularVelocity3D>(m_angularVel);
	m_angularVelOut.write();

	mx = 2.56 * rmx;
	my = 2.56 * rmy;
	mz = 2.56 * rmz;

	this->m_magnet.data.length(4);
	double a = sqrt(ax*ax + ay*ay + az*az);
	double m = sqrt(mx*mx + my*my + mz*mz);

	double hx = my*az - mz*ay;
	double hy = mz*ax - mx*az;
	double hz = mx*ay - my*ax;
	double h = sqrt(hx*hx + hy*hy + hz*hz);

	double rx = ay*hz - az*hy;
	double ry = az*hx - ax*hz;
	double rz = ax*hy - ay*hx;
	double r = sqrt(rx*rx + ry*ry + rz*rz);
	double heading = -atan2(hx / h, rx / r);

	m_magnet.data[0] = heading;
	m_magnet.data[1] = mx;
	m_magnet.data[2] = my;
	m_magnet.data[3] = mz;
	setTimestamp<RTC::TimedDoubleSeq>(m_magnet);
	m_magnetOut.write();


	if(m_debug) {
		static int counter;
		static double rad;
		static double old_wz;
		wz = m_angularVel.data.avz;
		double dt = (currentTime.getUsec() - m_timestamp.getUsec()) / (1000.0*1000.0);
		rad += (wz + old_wz) * dt /2;

		if((counter++) % 10 == 0) 
			std::cout << "rad: " << rad << ", heading: " << heading << std::endl;
		old_wz = wz;

		m_timestamp = currentTime;
	}
	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IMURazor9DOF::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IMURazor9DOF::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IMURazor9DOF::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IMURazor9DOF::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IMURazor9DOF::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void IMURazor9DOFInit(RTC::Manager* manager)
  {
    coil::Properties profile(imurazor9dof_spec);
    manager->registerFactory(profile,
                             RTC::Create<IMURazor9DOF>,
                             RTC::Delete<IMURazor9DOF>);
  }
  
};


