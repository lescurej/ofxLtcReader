#pragma once

#include "ofMain.h"
#include "ofxLtcReader.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

        void audioIn(float * input, int bufferSize, int nChannels);
    
    ofSoundStream snd;
    ofxLtcReader reader;
		
};
