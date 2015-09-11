#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    //Setup an audio device with ofSoundStream
    snd.listDevices();
    snd.setDeviceID(3); //'3' is my device ID, you have to replace with yours
    snd.setup(this, 0, 2, 44100, 256, 4); //0 outputs, 2 inputs, sample rate 44100Hz, buffer of 256, nb of buffer 4
    
    //Setup a ltc reader by passing a pointer to this ofSoundStream, expected frame rate of ltc = 30
    reader.setup(&snd, 30);

}

//--------------------------------------------------------------
void ofApp::update(){
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    //Print ltc values
    int h = reader.ltcHour();
    int m = reader.ltcMinute();
    int s = reader.ltcSecond();
    int f = reader.ltcFrame();
    string ltc = ofToString(h,2,'0') + ":" + ofToString(m,2,'0') + ":" + ofToString(s,2,'0') + ":" + ofToString(f,2,'0');
    ofSetColor(0);
    ofDrawBitmapString("LTC in : " + ltc, 20, 50);

}
//--------------------------------------------------------------

void ofApp::audioIn(float * input, int bufferSize, int nChannels){
    
    //Each time audioIn is called, call readLtc to update ltc values
    reader.readLtc(input, nChannels, 2);
    
}
