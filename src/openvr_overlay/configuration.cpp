#include "configuration.hpp"

#include <picojson.h>
#include <sys/stat.h>
#ifdef WIN32
#include <shlobj_core.h>
#else
#include <cstdlib>
#endif
#include <locale>
#include <codecvt>
#include <fstream>
#include <filesystem>

#ifdef WIN32
static std::wstring s_configPath;
#else
static std::filesystem::path s_configPath;
#endif
static bool s_directoriesExist = false;

static bool EnsureDirectoriesExist() {
	#ifdef WIN32
	PWSTR RootPath = NULL;
	if (S_OK != SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, NULL, &RootPath)) {
		CoTaskMemFree(RootPath);
		return false;
	}

	s_configPath = RootPath;
	CoTaskMemFree(RootPath);

	s_configPath += LR"(\FreeScuba)";
	if (CreateDirectoryW(s_configPath.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS) {
		return false;
	}
	s_configPath += L"\\config.json";
	#else 
	std::filesystem::path path;
	// there is the chance for a crash here if HOME is not set
	path = std::getenv("HOME");
	path /= ".config";
	path /= "freescuba";
	if(mkdir(path.c_str(), 0770) != 0){
		if(errno != EEXIST){
			return false;
		}
	}
	path /= "config.json";
	s_configPath = path;
	#endif

	return true;
}

inline void TryReadBool(bool& propToWriteTo, picojson::object objectToReadFrom, const char* valueName) {
	try {
		propToWriteTo = objectToReadFrom[valueName].get<bool>();
	} catch (std::runtime_error) {}
}
inline void TryReadDouble(double& propToWriteTo, picojson::object objectToReadFrom, const char* valueName) {
	try {
		propToWriteTo = objectToReadFrom[valueName].get<double>();
	} catch (std::runtime_error) {}
}
inline void TryReadFloat(float& propToWriteTo, picojson::object objectToReadFrom, const char* valueName) {
	try {
		propToWriteTo = (float) objectToReadFrom[valueName].get<double>();
	} catch (std::runtime_error) {}
}
inline void TryReadUint16(uint16_t& propToWriteTo, picojson::object objectToReadFrom, const char* valueName) {
	try {
		propToWriteTo = (uint16_t) objectToReadFrom[valueName].get<double>();
	} catch (std::runtime_error) {}
}
inline void TryReadUint8(uint8_t& propToWriteTo, picojson::object objectToReadFrom, const char* valueName) {
	try {
		propToWriteTo = (uint8_t) objectToReadFrom[valueName].get<double>();
	} catch (std::runtime_error) {}
}

void ReadPoseOffset(protocol::ContactGloveState_t::CalibrationData_t::PoseOffset_t& state, picojson::object& jsonObj) {

	try {
		picojson::object trackerOffsetRoot = jsonObj["pose"].get<picojson::object>();

		try {
			picojson::object positionRoot = trackerOffsetRoot["position"].get<picojson::object>();
			TryReadFloat(state.pos[0], positionRoot, "x");
			TryReadFloat(state.pos[1], positionRoot, "y");
			TryReadFloat(state.pos[2], positionRoot, "z");
			// state.pos.v[0] = positionRoot["x"].get<double>();
			// state.pos.v[1] = positionRoot["y"].get<double>();
			// state.pos.v[2] = positionRoot["z"].get<double>();
		}catch (std::runtime_error) {}

		try {
			picojson::object rotationRoot = trackerOffsetRoot["rotation"].get<picojson::object>();
			TryReadFloat(state.rot[0], rotationRoot, "x");
			TryReadFloat(state.rot[1], rotationRoot, "y");
			TryReadFloat(state.rot[2], rotationRoot, "z");
			TryReadFloat(state.rot[3], rotationRoot, "w");
			// state.rot.x = rotationRoot["x"].get<double>();
			// state.rot.y = rotationRoot["y"].get<double>();
			// state.rot.z = rotationRoot["z"].get<double>();
			// state.rot.w = rotationRoot["w"].get<double>();
		}catch (std::runtime_error) {}
	}catch (std::runtime_error) {}
}

void ReadJoystickCalibration(protocol::ContactGloveState_t::CalibrationData_t::JoystickCalibration_t& state, picojson::object& jsonObj) {

	try {
		picojson::object joystickRoot = jsonObj["joystick"].get<picojson::object>();

		TryReadFloat(state.threshold,		joystickRoot, "threshold");
		TryReadFloat(state.forwardAngle,	joystickRoot, "forward");
		// state.threshold		= (float) joystickRoot["threshold"].get<double>();
		// state.forwardAngle	= (float) joystickRoot["forward"].get<double>();

		TryReadUint16(state.XMax,			joystickRoot, "xmax");
		TryReadUint16(state.XMin,			joystickRoot, "xmin");
		TryReadUint16(state.YMax,			joystickRoot, "ymax");
		TryReadUint16(state.YMin,			joystickRoot, "ymin");
		TryReadUint16(state.XNeutral,		joystickRoot, "xneutral");
		TryReadUint16(state.YNeutral,		joystickRoot, "yneutral");

		// state.XMax			= (uint16_t) joystickRoot["xmax"].get<double>();
		// state.XMin			= (uint16_t) joystickRoot["xmin"].get<double>();
		// state.YMax			= (uint16_t) joystickRoot["ymax"].get<double>();
		// state.YMin			= (uint16_t) joystickRoot["ymin"].get<double>();
		// state.XNeutral		= (uint16_t) joystickRoot["xneutral"].get<double>();
		// state.YNeutral		= (uint16_t) joystickRoot["yneutral"].get<double>();
	} catch (std::runtime_error) {}
}

void ReadTriggerCalibration(protocol::ContactGloveState_t::CalibrationData_t::TriggerCalibration_t& state, picojson::object& jsonObj) {

	try {
		picojson::object triggerRoot = jsonObj["trigger"].get<picojson::object>();

		TryReadUint8(state.min, triggerRoot, "min");
		TryReadUint8(state.max, triggerRoot, "max");
	} catch (std::runtime_error) {}
}

void ReadFingerJointCalibration(protocol::ContactGloveState_t::FingerJointCalibrationData_t& state, picojson::object& jsonObj) {
	try {
		// state.rest		= (uint16_t) jsonObj["rest"].get<double>();
		// state.bend		= (uint16_t) jsonObj["bend"].get<double>();
		// state.close		= (uint16_t) jsonObj["close"].get<double>();
		#define READ_STATE(jointstate) \
		TryReadUint16(state.jointstate,	jsonObj, #jointstate);

		READ_STATE(rest)
		READ_STATE(bend)
		READ_STATE(close)
		READ_STATE(splayed)
		READ_STATE(horns)
		READ_STATE(peace)
		READ_STATE(flipoff)
		READ_STATE(point)
	} catch (std::runtime_error) {}
}

void ReadFingersCalibration(protocol::ContactGloveState_t::HandFingersCalibrationData_t& state, picojson::object& jsonObj) {

	try {
	picojson::object fingersRoot = jsonObj["fingers"].get<picojson::object>();

#define READ_FINGER_CALIBRATION(jointRoot, structInner)											\
	try {																						\
		picojson::object jointRoot##Json = fingersRoot[#jointRoot].get<picojson::object>();		\
		ReadFingerJointCalibration(state.structInner, jointRoot##Json);							\
	} catch (std::runtime_error) {}

	READ_FINGER_CALIBRATION(thumbBase,	thumbBase);
	READ_FINGER_CALIBRATION(thumbRoot,	thumb.proximal);
	READ_FINGER_CALIBRATION(thumbRoot2,	thumb.proximal2);
	READ_FINGER_CALIBRATION(thumbTip,	thumb.distal);
	READ_FINGER_CALIBRATION(indexRoot,	index.proximal);
	READ_FINGER_CALIBRATION(indexRoot2,	index.proximal2);
	READ_FINGER_CALIBRATION(indexTip,	index.distal);
	READ_FINGER_CALIBRATION(middleRoot,	middle.proximal);
	READ_FINGER_CALIBRATION(middleRoot2,	middle.proximal2);
	READ_FINGER_CALIBRATION(middleTip,	middle.distal);
	READ_FINGER_CALIBRATION(ringRoot,	ring.proximal);
	READ_FINGER_CALIBRATION(ringRoot2,	ring.proximal2);
	READ_FINGER_CALIBRATION(ringTip,	ring.distal);
	READ_FINGER_CALIBRATION(pinkyRoot,	pinky.proximal);
	READ_FINGER_CALIBRATION(pinkyRoot2,	pinky.proximal2);
	READ_FINGER_CALIBRATION(pinkyTip,	pinky.distal);

#undef READ_FINGER_CALIBRATION
	
	} catch (std::runtime_error) {}
}

void ReadSplayCalibration(protocol::ContactGloveState_t::CalibrationData_t::SplayCalibration_t& state, picojson::object& jsonObj){
	try{
		picojson::object splayRoot = jsonObj["splay"].get<picojson::object>();
		#define READ_FINGER_CALIBRATION(finger)\
		TryReadFloat(state.finger.scale, splayRoot, #finger"Scale"); \
		TryReadFloat(state.finger.offset, splayRoot, #finger"Offset");

		READ_FINGER_CALIBRATION(thumb)
		READ_FINGER_CALIBRATION(index)
		READ_FINGER_CALIBRATION(middle)
		READ_FINGER_CALIBRATION(ring)
		READ_FINGER_CALIBRATION(pinky)
		#undef READ_FINGER_CALIBRATION
	} catch (std::runtime_error) {}
}

void ReadGestures(protocol::ContactGloveState_t::CalibrationData_t::GestureCalibration_t& state, picojson::object& jsonObj) {

	try {
		picojson::object gesturesRoot = jsonObj["gestures"].get<picojson::object>();

		TryReadFloat(state.grip.activate, gesturesRoot, "grip_activate");
		TryReadFloat(state.grip.deactivate, gesturesRoot, "grip_deactivate");
		// state.grip.activate = (float)gesturesRoot["grip_activate"].get<double>();
		// state.grip.deactivate = (float)gesturesRoot["grip_deactivate"].get<double>();

		TryReadFloat(state.trigger.activate, gesturesRoot, "trigger_activate");
		TryReadFloat(state.trigger.deactivate, gesturesRoot, "trigger_deactivate");
		// state.trigger.activate = (float)gesturesRoot["trigger_activate"].get<double>();
		// state.trigger.deactivate = (float)gesturesRoot["trigger_deactivate"].get<double>();

		TryReadFloat(state.thumb.activate, gesturesRoot, "thumb_activate");
		TryReadFloat(state.thumb.deactivate, gesturesRoot, "thumb_deactivate");
		// state.thumb.activate = (float)gesturesRoot["thumb_activate"].get<double>();
		// state.thumb.deactivate = (float)gesturesRoot["thumb_deactivate"].get<double>();

	} catch (std::runtime_error) {}
}

void LoadConfiguration(AppState& state) {
	if (!s_directoriesExist) {
		s_directoriesExist = EnsureDirectoriesExist();
	}
	if (!s_directoriesExist) {
		throw std::runtime_error("Failed to creare config directory. Aborting...");
	}

	// Loads the config file from disk
	std::ifstream fileStream(s_configPath);

	if (fileStream.is_open()) {
		// Wrap in try catch as we shouldn't crash on invalid config
		try {
			picojson::value v;
			std::string err = picojson::parse(v, fileStream);
			if (!err.empty())
				throw std::runtime_error(err);

			auto rootObj = v.get<picojson::object>();

			// Read automatic launch
			// try {
			// 	state.doAutoLaunch = rootObj["doAutoLaunch"].get<bool>();
			// } catch (std::runtime_error) {}
			TryReadBool(state.doAutoLaunch, rootObj, "doAutoLaunch");

			// Load left glove config
			try {
				auto leftGloveObj = rootObj["left"].get<picojson::object>();
		
				ReadPoseOffset(state.gloveLeft.calibration.poseOffset, leftGloveObj);
				ReadJoystickCalibration(state.gloveLeft.calibration.joystick, leftGloveObj);
				ReadTriggerCalibration(state.gloveLeft.calibration.trigger, leftGloveObj);
				ReadFingersCalibration(state.gloveLeft.calibration.fingers, leftGloveObj);
				ReadSplayCalibration(state.gloveLeft.calibration.splay, leftGloveObj);
				ReadGestures(state.gloveLeft.calibration.gestures, leftGloveObj);
			} catch (std::runtime_error) {}

			// Load right glove config
			try {
				auto rightGloveObj = rootObj["right"].get<picojson::object>();
		
				ReadPoseOffset(state.gloveRight.calibration.poseOffset, rightGloveObj);
				ReadJoystickCalibration(state.gloveRight.calibration.joystick, rightGloveObj);
				ReadTriggerCalibration(state.gloveRight.calibration.trigger, rightGloveObj);
				ReadFingersCalibration(state.gloveRight.calibration.fingers, rightGloveObj);
				ReadSplayCalibration(state.gloveRight.calibration.splay, rightGloveObj);
				ReadGestures(state.gloveRight.calibration.gestures, rightGloveObj);
			} catch (std::runtime_error) {}

		} catch (std::runtime_error){}
		
		fileStream.close();
	}
}

void WritePoseCalibration(protocol::ContactGloveState_t::CalibrationData_t::PoseOffset_t& state, picojson::object& jsonObj) {

	picojson::object trackerOffsetRoot;
	picojson::object positionRoot;
	picojson::object rotationRoot;
	double buf;
	buf = state.pos[0]; positionRoot["x"].set<double>(buf);
	buf = state.pos[1]; positionRoot["y"].set<double>(buf);
	buf = state.pos[2]; positionRoot["z"].set<double>(buf);

	buf = state.rot[0]; rotationRoot["x"].set<double>(buf);
	buf = state.rot[1]; rotationRoot["y"].set<double>(buf);
	buf = state.rot[2]; rotationRoot["z"].set<double>(buf);
	buf = state.rot[3]; rotationRoot["w"].set<double>(buf);

	trackerOffsetRoot["position"].set<picojson::object>(positionRoot);
	trackerOffsetRoot["rotation"].set<picojson::object>(rotationRoot);

	jsonObj["pose"].set<picojson::object>(trackerOffsetRoot);
}

void WriteJoystickCalibration(protocol::ContactGloveState_t::CalibrationData_t::JoystickCalibration_t& state, picojson::object& jsonObj) {

	picojson::object joystickRoot;

	double buf = state.threshold; joystickRoot["threshold"].set<double>( buf );
	buf = state.XMax; joystickRoot["xmax"].set<double>( buf );
	buf = state.XMin; joystickRoot["xmin"].set<double>( buf );
	buf = state.YMax; joystickRoot["ymax"].set<double>( buf );
	buf = state.YMin; joystickRoot["ymin"].set<double>( buf );
	buf = state.XNeutral; joystickRoot["xneutral"].set<double>( buf );
	buf = state.YNeutral; joystickRoot["yneutral"].set<double>( buf );
	buf = state.forwardAngle; joystickRoot["forward"].set<double>( buf );

	jsonObj["joystick"].set<picojson::object>(joystickRoot);
}

void WriteTriggerCalibration(protocol::ContactGloveState_t::CalibrationData_t::TriggerCalibration_t& state, picojson::object& jsonObj){
	picojson::object triggerRoot;
	double buf = state.min;
	triggerRoot["min"].set<double>(buf);
	buf=state.max;
	triggerRoot["max"].set<double>(buf);

	jsonObj["trigger"].set<picojson::object>(triggerRoot);
}

void WriteFingerJointCalibration(protocol::ContactGloveState_t::FingerJointCalibrationData_t& state, picojson::object& jsonObj) {
	double buf;
	#define WRITE_STATE(jointstate) \
	buf = state.jointstate; \
	jsonObj[#jointstate].set<double>( buf );

	WRITE_STATE(rest)
	WRITE_STATE(bend)
	WRITE_STATE(close)
	WRITE_STATE(splayed)
	WRITE_STATE(horns)
	WRITE_STATE(peace)
	WRITE_STATE(flipoff)
	WRITE_STATE(point)
}

void WriteFingersCalibration(protocol::ContactGloveState_t::HandFingersCalibrationData_t& state, picojson::object& jsonObj) {

	picojson::object fingersRoot;

#define WRITE_FINGER_CALIBRATION(jointRoot, structInner)				\
	picojson::object jointRoot##Json;									\
	WriteFingerJointCalibration(state.structInner, jointRoot##Json);	\
	fingersRoot[#jointRoot].set<picojson::object>(jointRoot##Json);

	WRITE_FINGER_CALIBRATION(thumbBase,		thumbBase);
	WRITE_FINGER_CALIBRATION(thumbRoot,		thumb.proximal);
	WRITE_FINGER_CALIBRATION(thumbRoot2,		thumb.proximal2);
	WRITE_FINGER_CALIBRATION(thumbTip,		thumb.distal);
	WRITE_FINGER_CALIBRATION(indexRoot,		index.proximal);
	WRITE_FINGER_CALIBRATION(indexRoot2,		index.proximal2);
	WRITE_FINGER_CALIBRATION(indexTip,		index.distal);
	WRITE_FINGER_CALIBRATION(middleRoot,	middle.proximal);
	WRITE_FINGER_CALIBRATION(middleRoot2,	middle.proximal2);
	WRITE_FINGER_CALIBRATION(middleTip,		middle.distal);
	WRITE_FINGER_CALIBRATION(ringRoot,		ring.proximal);
	WRITE_FINGER_CALIBRATION(ringRoot2,		ring.proximal2);
	WRITE_FINGER_CALIBRATION(ringTip,		ring.distal);
	WRITE_FINGER_CALIBRATION(pinkyRoot,		pinky.proximal);
	WRITE_FINGER_CALIBRATION(pinkyRoot2,		pinky.proximal2);
	WRITE_FINGER_CALIBRATION(pinkyTip,		pinky.distal);

#undef WRITE_FINGER_CALIBRATION

	jsonObj["fingers"].set<picojson::object>(fingersRoot);
}

void WriteSplayCalibration(protocol::ContactGloveState_t::CalibrationData_t::SplayCalibration_t& state, picojson::object& jsonObj){
	picojson::object splayRoot;
	double buf;
	#define WRITE_FINGER_CALIBRATION(finger)\
	buf = state.finger.scale;\
	splayRoot[#finger"Scale"].set<double>(buf); \
	buf = state.finger.offset;\
	splayRoot[#finger"Offset"].set<double>(buf);

	WRITE_FINGER_CALIBRATION(thumb)
	WRITE_FINGER_CALIBRATION(index)
	WRITE_FINGER_CALIBRATION(middle)
	WRITE_FINGER_CALIBRATION(ring)
	WRITE_FINGER_CALIBRATION(pinky)
	#undef WRITE_FINGER_CALIBRATION
	jsonObj["splay"].set<picojson::object>(splayRoot);
}

void WriteThresholds(protocol::ContactGloveState_t::CalibrationData_t::GestureCalibration_t& state, picojson::object& jsonObj) {

	picojson::object gesturesRoot;

	double buf = state.grip.activate;
	gesturesRoot["grip_activate"].set<double>(buf);
	buf = state.grip.deactivate;
	gesturesRoot["grip_deactivate"].set<double>(buf);

	buf = state.trigger.activate;
	gesturesRoot["trigger_activate"].set<double>(buf);
	buf = state.trigger.deactivate;
	gesturesRoot["trigger_deactivate"].set<double>(buf);

	buf = state.thumb.activate;
	gesturesRoot["thumb_activate"].set<double>(buf);
	buf = state.thumb.deactivate;
	gesturesRoot["thumb_deactivate"].set<double>(buf);

	jsonObj["gestures"].set<picojson::object>(gesturesRoot);
}

void SaveConfiguration(AppState& state) {
	if (!s_directoriesExist) {
		s_directoriesExist = EnsureDirectoriesExist();
	}
	if (!s_directoriesExist) {
		throw std::runtime_error("Failed to creare config directory. Aborting...");
	}

	// Saves the config file to disk
	std::ofstream fileStream(s_configPath);
	if (fileStream.is_open()) {

		picojson::object config;

		picojson::object gloveLeftConfig;

		// Write props
		WritePoseCalibration(state.gloveLeft.calibration.poseOffset, gloveLeftConfig);
		WriteJoystickCalibration(state.gloveLeft.calibration.joystick, gloveLeftConfig);
		WriteTriggerCalibration(state.gloveLeft.calibration.trigger, gloveLeftConfig);
		WriteFingersCalibration(state.gloveLeft.calibration.fingers, gloveLeftConfig);
		WriteSplayCalibration(state.gloveLeft.calibration.splay, gloveLeftConfig);
		WriteThresholds(state.gloveLeft.calibration.gestures, gloveLeftConfig);

		picojson::object gloveRightConfig;
		
		// Write props
		WritePoseCalibration(state.gloveRight.calibration.poseOffset, gloveRightConfig);
		WriteJoystickCalibration(state.gloveRight.calibration.joystick, gloveRightConfig);
		WriteTriggerCalibration(state.gloveRight.calibration.trigger, gloveRightConfig);
		WriteFingersCalibration(state.gloveRight.calibration.fingers, gloveRightConfig);
		WriteSplayCalibration(state.gloveRight.calibration.splay, gloveRightConfig);
		WriteThresholds(state.gloveRight.calibration.gestures, gloveRightConfig);

		config["left"].set<picojson::object>(gloveLeftConfig);
		config["right"].set<picojson::object>(gloveRightConfig);

		// Write do automatic launch
		config["doAutoLaunch"].set<bool>(state.doAutoLaunch);

		picojson::value rootV;
		rootV.set<picojson::object>(config);

		fileStream << rootV.serialize(true);

		fileStream.close();
	}
}