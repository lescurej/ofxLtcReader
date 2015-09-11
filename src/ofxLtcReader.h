/*
 ofxLtcReader.h
 ofxLtcReader
 
 Copyright (c) 2015, Johan Lescure. All rights reserved.
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __ofxLtcReader__
#define __ofxLtcReader__

#include "ofMain.h"

#include "ofxLtcReader.h"

class ofxLtcReader {
public:
    ofxLtcReader();
    ~ofxLtcReader();
    
    bool    setup(ofSoundStream *snd, int frameRate);
    
    bool    readLtc(float * input, int nbOfChannel, int channelId);
    //Put readLtc() in audioIn() listener
    //Input is the sound buffer
    //nbOfChannel is the total number oh channel
    //channelId is this index of the ltc channel
    //ex : ltc on channel 2 on a sound device with a total of 8 inputs --> readLtc(input, 8, 2);
    
    int     ltcFrame();
    int     ltcSecond();
    int     ltcMinute();
    int     ltcHour();
    
protected:
    
    
    
private:
    
    
    typedef float sample_t;
    typedef double timeu;
    
    typedef struct SMPTETime {
        // these are only set when compiled with ENABLE_DATE
        char timezone[6];
        unsigned char years;
        unsigned char months;
        unsigned char days;
        //
        unsigned char hours;
        unsigned char mins;
        unsigned char secs;
        unsigned char frame;
    } SMPTETime;
    
    typedef enum
    {
        FRF_NONE = 0,
        FRF_DROP_FRAMES = 1, ///< use drop frame timecode
        FRF_UNUSED = 2,
        FRF_SAMPLERATE = 4, ///< has sample-rate info
        FRF_OFFA = 8, ///< has audio frame offset
        FRF_OFFV = 16, ///< has video frame offset
        FRF_LAST = 32
    } FRFlags;
    
    typedef struct SMPTEFrame {
        unsigned int frameUnits:4;
        unsigned int user1:4;
        unsigned int frameTens:2;
        unsigned int dfbit:1;
        unsigned int colFrm:1;
        unsigned int user2:4;
        unsigned int secsUnits:4;
        unsigned int user3:4;
        unsigned int secsTens:3;
        unsigned int biphaseMarkPhaseCorrection:1;
        unsigned int user4:4;
        unsigned int minsUnits:4;
        unsigned int user5:4;
        unsigned int minsTens:3;
        unsigned int binaryGroupFlagBit1:1;
        unsigned int user6:4;
        unsigned int hoursUnits:4;
        unsigned int user7:4;
        unsigned int hoursTens:2;
        unsigned int reserved:1;
        unsigned int binaryGroupFlagBit2:1;
        unsigned int user8:4;
        unsigned int syncWord:16;
    } SMPTEFrame;
    
    typedef struct FrameRate
    {
        int num; ///< numerator; rational framerate: numerator/denominator
        int den; ///< denominator; rational framerate: numerator/denominator
        int flags; ///< combination of FRFlags;
        int samplerate; ///< audio-samplerate for conversion.
        long long int aoffset; ///< user-data: offset in audio-frames
        long int voffset; ///< user-data: offset in video-frames
    } FrameRate;
    
    typedef struct SMPTEFrameExt {
        SMPTEFrame base; ///< the SMPTE decoded from the audio
        int delayed; ///< detected jitter in LTC-framerate/80 unit(s) - bit count in LTC frame.
        long int startpos; ///< the approximate sample in the stream corresponding to the start of the LTC SMPTE frame.
        long int endpos; ///< the sample in the stream corresponding to the end of the LTC SMPTE frame.
    } SMPTEFrameExt;
    
    struct SMPTEDecoder {
        int sampleRate;
        FrameRate fps;
        
        SMPTEFrameExt* queue;
        int qLen;
        int qReadPos;
        int qWritePos;
        
        int firstFrame;
        int lastFrame;
        int errorCnt;
        
        unsigned char biphaseToBinaryState;
        unsigned char biphaseToBinaryPrev;
        unsigned char soundToBiphaseState;
        int soundToBiphaseCnt;		// counts the samples in the current period, resets every period
        int soundToBiphasePeriod;	// length of a period (tracks speed variations)
        int soundToBiphaseLimit;	// specifies when a state-change is considered biphase-clock or 2*biphase-clock
        sample_t soundToBiphaseMin;
        sample_t soundToBiphaseMax;
        unsigned short decodeSyncWord;
        SMPTEFrame decodeFrame;
        int decodeBitCnt;
        long int decodeFrameStartPos;
        
        int correctJitter;
        
        timeu samplesToSeconds;        
    };
    
    SMPTEDecoder *decoder;
    FrameRate *fps;
    SMPTEFrameExt frame;
    size_t total;
    size_t prevRead;
    size_t frameStart;
    size_t frameStartFudge;
    int errors;
    sample_t *tab;
    int bufferSize;
    SMPTETime stime;
    
    int audio_to_biphase(SMPTEDecoder *d, sample_t *sound, unsigned char *biphase, unsigned long *offset, int size);
    int biphase_decode(SMPTEDecoder *d, unsigned char *biphase, unsigned long *offset_in, unsigned char *bits, unsigned long *offset_out, int size);
    int ltc_decode(SMPTEDecoder *d, unsigned char *bits, unsigned long *offset, long int posinfo, int size);
    long int frame_to_vf(SMPTEFrame *f, FrameRate *fps);
    long int FR_smpte2vf (FrameRate *fr, int f, int s, int m, int h, int overflow);
    long int FR_drop_frames (FrameRate *fr, int f, int s, int m, int h);
    int SMPTEDecoderWrite(SMPTEDecoder *d, sample_t *buf, int size, long int posinfo);
    int SMPTEDecoderRead(SMPTEDecoder* decoder, SMPTEFrameExt* frame);
    SMPTEDecoder* SMPTEDecoderCreate(int sampleRate, FrameRate *fps, int queueLen, int correctJitter);
    FrameRate * FR_create(int num, int den, int flags);
    void FR_setdbl(FrameRate *fr, double fps, int mode);
    void FR_setratio(FrameRate *fr, int num, int den);
    void FR_free (FrameRate *f);
    int SMPTEFrameToTime(SMPTEFrame* frame, SMPTETime* stime);
    int SMPTEDecoderFrameToMillisecs(SMPTEDecoder* d, SMPTEFrameExt* frame, int* timems);
    static int SMPTEFreeDecoder(SMPTEDecoder *d);

    
};

#endif /* defined(__ofLtcReader__) */

