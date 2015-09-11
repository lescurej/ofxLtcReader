ofxLtcReader
============

A ltc reader for openframeworks

Usage
-----

Before all, you must setup an ofSoundStream to get a sound input.
Then : 
ofApp.h:

    #import "ofxLtcReader.h"

    ofxLtcReader reader;

ofApp.cpp:

    //setup an ofSoundStream
    snd.setDeviceID(0);
    snd.setup(this, 0, 2, 44100, 256, 4);
    //setup ofxLtcReader
    reader.setup(&snd, 30);

In audioIn listener put :

    void ofApp::audioIn(float * input, int bufferSize, int nChannels){
        reader.readLtc(input, nChannels, 2);
    }

Then you can read hours, minutes, seconds and frames everywhere in your code by calling :

    reader.ltcHour();
    reader.ltcMinute();
    reader.ltcSecond();
    reader.ltcFrame();
    
Credits and License
-------------------

ofxLtcReader was written by [Johan Lescure], September 2015.
Based on libltcsmpte http://ltcsmpte.sourceforge.net/
