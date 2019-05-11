// VEX V5 C++ Project
#include "vex.h"
#include <list>
#include <algorithm>
#include <fstream>
#include <iostream>
using namespace vex;

//#region config_globals

//#endregion config_globals

////////////////////////////////////////////////////////////////////////////////

class velocityRecordedMotor : public vex::motor {
    private:
    
    struct dataslice {
        uint32_t timestamp;
        double normalizedVelocity;
        velocityUnits velocityType;
        
        dataslice() {
            
        }
        
        dataslice(uint32_t time, double vel, velocityUnits units) {
            timestamp = time;
            normalizedVelocity = vel;
            velocityType = units;
        }
    };
    
    std::list<dataslice> data;
    std::list<dataslice>::iterator playbackMarker;
    
    const char* logFileName;
    
    uint32_t lastRecordingTime;
    uint32_t operationBeginTime;
    
    enum recordingState {none, recording, playback};
    recordingState myState = none;
    
    public:
    
    velocityRecordedMotor (int32_t index, vex::gearSetting gears, bool reversed, const char* logFileName)
      : vex::motor (index,gears,reversed) {
        this->logFileName = logFileName;
        lastRecordingTime = vex::timer::system();
    }
    
    void spin (vex::directionType dir, double velocity, vex::velocityUnits units) {
        uint32_t invocationTime = timer::system();
        if (myState == recording && invocationTime > lastRecordingTime) {
            vex::motor::spin(dir,velocity,units);
            if (dir == directionType::rev) velocity = -velocity;
            data.emplace_back(invocationTime - operationBeginTime,velocity,units);
            lastRecordingTime = invocationTime;
        } else if (myState == playback) {
            if (playbackMarker == data.end()) {
                stop(brakeType::hold);
            } else if (invocationTime >= operationBeginTime + playbackMarker->timestamp) {
                vex::motor::spin(vex::directionType::fwd, playbackMarker->normalizedVelocity, playbackMarker->velocityType);
                playbackMarker++;
            }
        } else if (myState == none) {
            vex::motor::spin(dir,velocity,units);
        }
    }
    
    void spin (directionType dir, double velocity, percentUnits units) {
        spin(dir,velocity,velocityUnits::pct);
    }
    
    void enableRecording() {
        lastRecordingTime = operationBeginTime = timer::system();
        data.clear();
        data.emplace_back(0,0,velocityUnits::pct);
        myState = recording;
    }
    
    bool isRecording() {
        return myState == recording;
    }
    
    bool isPlayback() {
        return myState == playback;
    }
    
    bool donePlayback() {
        return playbackMarker == data.end();
    }
    
    bool isIdle() {
        return myState == none;
    }
    
    void disableRecording() {
        if (myState == recording) {
            myState = none;
        }
    }
    
    void enablePlayback() {
        playbackMarker = data.begin();
        operationBeginTime = vex::timer::system();
        myState = playback;
    }
    
    void disablePlayback() {
        if (myState == playback) {
            myState = none;
        }
    }
    
    void disableRecordingOrPlayback () {
        myState = none;
    }
    
    void saveRecording() {
        disableRecording();
        std::ofstream outputFile(logFileName, std::ofstream::out | std::ofstream::trunc | std::ofstream::binary);
        for (std::list<dataslice>::iterator it = data.begin(); it != data.end(); ++it) {
            outputFile.write((char*)&(it->timestamp),sizeof(dataslice::timestamp));
            outputFile.write((char*)&(it->normalizedVelocity),sizeof(dataslice::normalizedVelocity));
            outputFile.write((char*)&(it->velocityType),sizeof(dataslice::velocityType));
            outputFile.flush();
        }
        outputFile.close();
    }
    
    void loadRecording () {
        disableRecording();
        data.clear();
        std::ifstream inputFile(logFileName, std::ifstream::in | std::ifstream::binary);
        while (!inputFile.eof()) {
            dataslice nextInput;
            inputFile.read((char*)&(nextInput.timestamp),sizeof(dataslice::timestamp));
            inputFile.read((char*)&(nextInput.normalizedVelocity),sizeof(dataslice::normalizedVelocity));
            inputFile.read((char*)&(nextInput.velocityType),sizeof(dataslice::velocityType));
            data.push_back(nextInput);
        }
    }
    
    ~velocityRecordedMotor() {
        
    }
};

////////////////////////////////////////////////////////////////////////////////


velocityRecordedMotor recordedleftMotorFront(vex::PORT1, vex::gearSetting::ratio18_1, false, "leftMotorFrontVlog.dat");
velocityRecordedMotor recordedleftMotorMiddle(vex::PORT2, vex::gearSetting::ratio18_1, false, "leftMotorMiddleVlog.dat");
velocityRecordedMotor recordedleftMotorBack(vex::PORT3, vex::gearSetting::ratio18_1, false, "leftMotoBackVlog.dat");
velocityRecordedMotor recordedrightMotorFront(vex::PORT4, vex::gearSetting::ratio18_1, true, "rightMotorFrontVlog.dat");
velocityRecordedMotor recordedrightMotorMiddle(vex::PORT5, vex::gearSetting::ratio18_1, true, "rightMotorMiddleVlog.dat");
velocityRecordedMotor recordedrightMotorBack(vex::PORT6, vex::gearSetting::ratio18_1, true, "rightMotorBackVlog.dat");


int main(void) {
    // Start here
    bool AlastPressed = false;
    bool BlastPressed = false;
    bool XlastPressed = false;
    bool YlastPressed = false;
    bool checkForPlaybackEnd = false;
    while (true) {
        if (!AlastPressed && con.ButtonA.pressing()) {
            if (!recordedleftMotorFront.isRecording()) {
                recordedleftMotorFront.disableRecordingOrPlayback();
                recordedrightMotorFront.disableRecordingOrPlayback();
                recordedleftMotorMiddle.disableRecordingOrPlayback();
                recordedrightMotorMiddle.disableRecordingOrPlayback();
                recordedleftMotorBack.disableRecordingOrPlayback();
                recordedrightMotorBack.disableRecordingOrPlayback();
                recordedleftMotorFront.enableRecording();
                recordedrightMotorFront.enableRecording();
                recordedleftMotorMiddle.enableRecording();
                recordedrightMotorMiddle.enableRecording();
                recordedleftMotorBack.enableRecording();
                recordedrightMotorBack.enableRecording();
                
                con.Screen.setCursor(1,1);
                con.Screen.print("Recording...         ");
            } else {
                recordedleftMotorFront.disableRecordingOrPlayback();
                recordedrightMotorFront.disableRecordingOrPlayback();
                recordedleftMotorMiddle.disableRecordingOrPlayback();
                recordedrightMotorMiddle.disableRecordingOrPlayback();
                recordedleftMotorBack.disableRecordingOrPlayback();
                recordedrightMotorBack.disableRecordingOrPlayback();
                con.Screen.clearLine(1);
            }
            AlastPressed = true;
        } else if (!BlastPressed && con.ButtonB.pressing()) {
            if (!recordedleftMotorFront.isPlayback()) {
                recordedleftMotorFront.disableRecordingOrPlayback();
                recordedrightMotorFront.disableRecordingOrPlayback();
                recordedleftMotorMiddle.disableRecordingOrPlayback();
                recordedrightMotorMiddle.disableRecordingOrPlayback();
                recordedleftMotorBack.disableRecordingOrPlayback();
                recordedrightMotorBack.disableRecordingOrPlayback();
                recordedleftMotorFront.enablePlayback();
                recordedrightMotorFront.enablePlayback();
                recordedleftMotorMiddle.enablePlayback();
                recordedrightMotorMiddle.enablePlayback();
                recordedleftMotorBack.enablePlayback();
                recordedrightMotorBack.enablePlayback();
                con.Screen.setCursor(1,1);
                con.Screen.print("Playback...        ");
                checkForPlaybackEnd = true;
            } else {
                recordedleftMotorFront.disableRecordingOrPlayback();
                recordedrightMotorFront.disableRecordingOrPlayback();
                recordedleftMotorMiddle.disableRecordingOrPlayback();
                recordedrightMotorMiddle.disableRecordingOrPlayback();
                recordedleftMotorBack.disableRecordingOrPlayback();
                recordedrightMotorBack.disableRecordingOrPlayback();
                con.Screen.clearLine(1);
                checkForPlaybackEnd = false;
            }
            BlastPressed = true;
        } else if (!XlastPressed && con.ButtonX.pressing()) {
            recordedleftMotorFront.disableRecordingOrPlayback();
            recordedrightMotorFront.disableRecordingOrPlayback();
            recordedleftMotorMiddle.disableRecordingOrPlayback();
            recordedrightMotorMiddle.disableRecordingOrPlayback();
            recordedleftMotorBack.disableRecordingOrPlayback();
            recordedrightMotorBack.disableRecordingOrPlayback();
            con.Screen.setCursor(1,1);
            uint32_t startLoadTime = timer::system();
            con.Screen.print("Saving...           ");
            recordedleftMotorFront.stop();
            recordedleftMotorMiddle.stop();
            recordedleftMotorBack.stop();
            recordedrightMotorFront.stop();
            recordedrightMotorMiddle.stop();
            recordedrightMotorBack.stop();
            recordedleftMotorFront.saveRecording();
            recordedleftMotorMiddle.saveRecording();
            recordedleftMotorBack.saveRecording();
            recordedrightMotorFront.saveRecording();
            recordedrightMotorMiddle.saveRecording();
            recordedrightMotorBack.saveRecording();
            while (timer::system() < startLoadTime + 1000);
            con.Screen.clearLine(1);
            XlastPressed = true;
        } else if (!YlastPressed && con.ButtonY.pressing()) {
            recordedleftMotorFront.disableRecordingOrPlayback();
            recordedrightMotorFront.disableRecordingOrPlayback();
            recordedleftMotorMiddle.disableRecordingOrPlayback();
            recordedrightMotorMiddle.disableRecordingOrPlayback();
            recordedleftMotorBack.disableRecordingOrPlayback();
            recordedrightMotorBack.disableRecordingOrPlayback();
            con.Screen.setCursor(1,1);
            uint32_t startLoadTime = timer::system();
            con.Screen.print("Loading...           ");
            recordedleftMotorFront.stop();
            recordedleftMotorMiddle.stop();
            recordedleftMotorBack.stop();
            recordedrightMotorFront.stop();
            recordedrightMotorMiddle.stop();
            recordedrightMotorBack.stop();
            recordedleftMotorFront.loadRecording();
            recordedleftMotorMiddle.loadRecording();
            recordedleftMotorBack.loadRecording();
            recordedrightMotorFront.loadRecording();
            recordedrightMotorMiddle.loadRecording();
            recordedrightMotorBack.loadRecording();
            while (timer::system() < startLoadTime + 1000);
            con.Screen.clearLine(1);
            YlastPressed = true;
        }
        AlastPressed = AlastPressed && con.ButtonA.pressing();
        BlastPressed = BlastPressed && con.ButtonB.pressing();
        XlastPressed = XlastPressed && con.ButtonX.pressing();
        YlastPressed = YlastPressed && con.ButtonY.pressing();
        
        recordedleftMotorFront.spin(vex::directionType::fwd,con.Axis3.position(),vex::velocityUnits::pct);
        recordedleftMotorMiddle.spin(vex::directionType::fwd,con.Axis3.position(),vex::velocityUnits::pct);
        recordedleftMotorBack.spin(vex::directionType::fwd,con.Axis3.position(),vex::velocityUnits::pct);
        recordedrightMotorFront.spin(vex::directionType::fwd,con.Axis2.position(),vex::velocityUnits::pct);
        recordedrightMotorMiddle.spin(vex::directionType::fwd,con.Axis2.position(),vex::velocityUnits::pct);
        recordedrightMotorBack.spin(vex::directionType::fwd,con.Axis2.position(),vex::velocityUnits::pct);
        
        if (checkForPlaybackEnd && recordedleftMotorFront.donePlayback() && recordedleftMotorMiddle.donePlayback() && recordedleftMotorBack.donePlayback() 
        && recordedrightMotorFront.donePlayback() && recordedrightMotorMiddle.donePlayback() && recordedrightMotorBack.donePlayback()) {
            checkForPlaybackEnd = false;
            con.Screen.setCursor(1,1);
            con.Screen.print("Done playback    ");
            recordedleftMotorFront.disablePlayback();
            recordedleftMotorMiddle.disablePlayback();
            recordedleftMotorBack.disablePlayback();
            recordedrightMotorFront.disablePlayback();
            recordedrightMotorMiddle.disablePlayback();
            recordedrightMotorBack.disablePlayback();
        }
        
        task::sleep(30);
    }
}
