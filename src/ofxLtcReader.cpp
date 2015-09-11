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


/*
 ofxLtcReader
 
An audio LTC reader for openframeworks

Don't know if it's working on windows...
 
*/


#include "ofxLtcReader.h"

/** turn a numeric literal into a hex constant
 *  (avoids problems with leading zeroes)
 *  8-bit constants max value 0x11111111, always fits in unsigned long
 */
#define HEX__(n) 0x##n##LU
#define TIME_CODE_STRING_SIZE	12
#define QUEUE_LENGTH 2
#define DF_DELIMITER ';' // drop frame timecode frame delimiter
#define NDF_DELIMITER ':' // non-drop frame timecode frame delimiter

/**
 * 8-bit conversion function
 */
#define B8__(x) ((x&0x0000000FLU)?1:0)	\
+((x&0x000000F0LU)?2:0)	 \
+((x&0x00000F00LU)?4:0)	 \
+((x&0x0000F000LU)?8:0)	 \
+((x&0x000F0000LU)?16:0) \
+((x&0x00F00000LU)?32:0) \
+((x&0x0F000000LU)?64:0) \
+((x&0xF0000000LU)?128:0)

#define B8(d) ((unsigned char)B8__(HEX__(d)))

/** for upto 16-bit binary constants, MSB first */
#define B16(dmsb,dlsb) (((unsigned short)B8(dmsb)<<8) + B8(dlsb))

/** for upto 32-bit binary constants, MSB first */
#define B32(dmsb,db2,db3,dlsb) (((unsigned long)B8(dmsb)<<24) \
+ ((unsigned long)B8(db2)<<16) \
+ ((unsigned long)B8(db3)<<8)  \
+ B8(dlsb))

/** turn a numeric literal into a hex constant
 *(avoids problems with leading zeroes)
 * 8-bit constants max value 0x11111111, always fits in unsigned long
 */
#define HEX__(n) 0x##n##LU

/** 8-bit conversion function */
#define B8__(x) ((x&0x0000000FLU)?1:0)	\
+((x&0x000000F0LU)?2:0)  \
+((x&0x00000F00LU)?4:0)  \
+((x&0x0000F000LU)?8:0)  \
+((x&0x000F0000LU)?16:0) \
+((x&0x00F00000LU)?32:0) \
+((x&0x0F000000LU)?64:0) \
+((x&0xF0000000LU)?128:0)


/** for upto 8-bit binary constants */
#define B8(d) ((unsigned char)B8__(HEX__(d)))

/** for upto 16-bit binary constants, MSB first */
#define B16(dmsb,dlsb) (((unsigned short)B8(dmsb)<<8) + B8(dlsb))

/** for upto 32-bit binary constants, MSB first */
#define B32(dmsb,db2,db3,dlsb) (((unsigned long)B8(dmsb)<<24)	 \
+ ((unsigned long)B8(db2)<<16) \
+ ((unsigned long)B8(db3)<<8)  \
+ B8(dlsb))

/* Example usage:
 * B8(01010101) = 85
 * B16(10101010,01010101) = 43605
 * B32(10000000,11111111,10101010,01010101) = 2164238933
 */

#define LTC_FRAME_BIT_COUNT	80

#define WRITE_DECODER_BIPHASE_DIAGNOSTICS(A, B, C, D, E)
#define WRITE_DECODER_BITS_DIAGNOSTICS(A, B, C, D, E)


ofxLtcReader::ofxLtcReader() : decoder(NULL), fps(NULL), total(0), prevRead(0), frameStart(0), frameStartFudge(0), errors(0), tab(NULL), bufferSize(0)
{
   
}

ofxLtcReader::~ofxLtcReader()
{
    SMPTEFreeDecoder(decoder);
}

//--------------------------------------------------------------
bool ofxLtcReader::setup(ofSoundStream *snd, int frameRate){

    bufferSize = snd->getBufferSize();
    tab = new sample_t[bufferSize];
    int sampleRate = snd->getSampleRate();
    double framesPerSec = frameRate;
    sample_t sound[bufferSize];
    size_t n;
    int printMissingFrameInfo;
    fps = FR_create(1, 1, FRF_NONE);
    FR_setdbl(fps, framesPerSec, 1); // auto-detect drop-frames
    double ltcFrameSamples = sampleRate / framesPerSec;
    double ltcFramePlusFudge = (ltcFrameSamples * 101 / 100);
    
    size_t ltcFrameSize = (size_t)ceil(ltcFrameSamples) * sizeof(sample_t);
    
    char timeCodeString[TIME_CODE_STRING_SIZE];
    prevRead = (size_t)round(ltcFrameSamples);
    decoder = SMPTEDecoderCreate(sampleRate, fps, QUEUE_LENGTH, 0);
    FR_free(fps);
    return 1;
}

bool ofxLtcReader::readLtc(float * input, int numberOfChannel, int channelId){
    if((channelId > numberOfChannel) || (channelId == 0)) return 0;
    for(int i = 0; i < bufferSize ; i++){
        tab[i] = input[i*numberOfChannel + (channelId-1)];
    }
    SMPTEDecoderWrite(decoder, tab, bufferSize, 0);
    while (SMPTEDecoderRead(decoder, &frame)) {
        prevRead = frame.endpos;
    }
    if(SMPTEFrameToTime(&frame.base, &stime)) return 1;
    return -1;
}

int ofxLtcReader::ltcFrame(){
    return stime.frame;
}

int ofxLtcReader::ltcSecond(){
    return stime.secs;
}

int ofxLtcReader::ltcMinute(){
    return stime.mins;
}

int ofxLtcReader::ltcHour(){
    return stime.hours;
}

int ofxLtcReader::audio_to_biphase(SMPTEDecoder *d, sample_t *sound, unsigned char *biphase, unsigned long *offset, int size) {
    int i;
    int j = 0;
#ifdef SAMPLE_IS_INTEGER
    int max_threshold, min_threshold;
#else
    sample_t max_threshold, min_threshold;
#endif
#ifdef DIAGNOSTICS_OUTPUT
    int c;
#endif
    
    for (i = 0 ; i < size ; i++) {
        /* track minimum and maximum values */
#ifdef SAMPLE_IS_UNSIGNED
        d->soundToBiphaseMin = SAMPLE_CENTER - (((SAMPLE_CENTER - d->soundToBiphaseMin) * 15) / 16);
        d->soundToBiphaseMax = SAMPLE_CENTER + (((d->soundToBiphaseMax - SAMPLE_CENTER) * 15) / 16);
#else
        d->soundToBiphaseMin = d->soundToBiphaseMin * 15 / 16;
        d->soundToBiphaseMax = d->soundToBiphaseMax * 15 / 16;
        
#endif
        
        if (sound[i] < d->soundToBiphaseMin)
            d->soundToBiphaseMin = sound[i];
        if (sound[i] > d->soundToBiphaseMax)
            d->soundToBiphaseMax = sound[i];
        
        /* set the thresholds for hi/lo state tracking */
#ifdef SAMPLE_IS_UNSIGNED
        min_threshold = SAMPLE_CENTER - (((SAMPLE_CENTER - d->soundToBiphaseMin) * 8) / 16);
        max_threshold = SAMPLE_CENTER + (((d->soundToBiphaseMax - SAMPLE_CENTER) * 8) / 16);
#else
        min_threshold =  d->soundToBiphaseMin * 8 / 16;
        max_threshold =  d->soundToBiphaseMax * 8 / 16;
#endif
        
        if (
            (  d->soundToBiphaseState && (sound[i] > max_threshold) ) // Check for a biphase state change
            || ( !d->soundToBiphaseState && (sound[i] < min_threshold) ) //
            ) {
            // There is a state change at sound[i]
            // If the sample count has risen above the biphase length limit
            if (d->soundToBiphaseCnt > d->soundToBiphaseLimit) {
                // single state change within a biphase priod => will decode to a 0
                offset[j] = i;
                biphase[j++] = d->soundToBiphaseState;
                offset[j] = i;
                biphase[j++] = d->soundToBiphaseState;
                
            } else {
                // "short" state change covering half a period
                // => together with the next or previous state change, will decode to a 1
                offset[j] = i;
                biphase[j++] = d->soundToBiphaseState;
                d->soundToBiphaseCnt *= 2;
            }
            
            /* track speed variations */
            // As this is only executed at a state change,
            // d->soundToBiphaseCnt is an accurate representation of the current period length.
            d->soundToBiphasePeriod = (d->soundToBiphasePeriod*3 + d->soundToBiphaseCnt) / 4;
            
            /* This limit specifies when a state-change is
             * considered biphase-clock or 2*biphase-clock.
             * The relation with period has been determined
             * through trial-and-error */
            d->soundToBiphaseLimit = (d->soundToBiphasePeriod * 14) / 16;
            
            d->soundToBiphaseCnt = 0;
            d->soundToBiphaseState = !d->soundToBiphaseState;
        }
        
        d->soundToBiphaseCnt++;
        
#ifdef DIAGNOSTICS_OUTPUT
        c = -1;
        d->diagnosticsData[++c] = (diagnostics_t)d->diagnosticsPos;
        d->diagnosticsData[++c] = (diagnostics_t)sound[i]-SAMPLE_CENTER;
        d->diagnosticsData[++c] = (diagnostics_t)d->soundToBiphaseMin-SAMPLE_CENTER;
        d->diagnosticsData[++c] = (diagnostics_t)d->soundToBiphaseMax-SAMPLE_CENTER;
        d->diagnosticsData[++c] = (diagnostics_t)min_threshold-SAMPLE_CENTER;
        d->diagnosticsData[++c] = (diagnostics_t)max_threshold-SAMPLE_CENTER;
        d->diagnosticsData[++c] = (diagnostics_t)-(d->soundToBiphaseState*20)+10;
        d->diagnosticsData[++c] = (diagnostics_t)d->soundToBiphaseCnt;
        d->diagnosticsData[++c] = (diagnostics_t)d->soundToBiphaseLimit;
        d->diagnosticsData[++c] = (diagnostics_t)d->soundToBiphasePeriod;
        
        fwrite(&d->diagnosticsData, sizeof(diagnostics_t), DIAGNOSTICS_DATA_SIZE, d->diagnosticsFile);
        
        d->diagnosticsPos++;
#endif
        
    }
    return j;
}

int ofxLtcReader::biphase_decode(SMPTEDecoder *d, unsigned char *biphase, unsigned long *offset_in, unsigned char *bits, unsigned long *offset_out, int size) {
    int i;
    int j = 0;
    
    for (i = 0 ; i < size ; i++) {
        offset_out[j] = offset_in[i];
        if (biphase[i] == d->biphaseToBinaryPrev) {
            d->biphaseToBinaryState = 1;
            bits[j++] = 0;
        } else {
            d->biphaseToBinaryState = 1 - d->biphaseToBinaryState;
            if (d->biphaseToBinaryState == 1)
                bits[j++] = 1;
        }
        d->biphaseToBinaryPrev = biphase[i];
    }
    return j;
}

int ofxLtcReader::ltc_decode(SMPTEDecoder *d, unsigned char *bits, unsigned long *offset, long int posinfo, int size) {
    int i;
    int bitNum, bitSet, bytePos;
    
    for (i = 0 ; i < size ; i++) {
        if (d->decodeBitCnt == 0) {
            memset(&d->decodeFrame, 0, sizeof(SMPTEFrame));
            
            if (i > 0) {
                d->decodeFrameStartPos = posinfo + offset[i-1];
            }
            else {
                d->decodeFrameStartPos = posinfo + offset[i] - d->soundToBiphasePeriod; // d->soundToBiphasePeriod is an approximation
            }
            
        }
        
#ifdef TRACK_DECODE_FRAME_BIT_OFFSETS
        d->decodeFrameBitOffsets[d->decodeBitCnt] = posinfo + offset[i];
#endif
        
        d->decodeSyncWord <<= 1;
        if (bits[i]) {
            
            d->decodeSyncWord |= B16(00000000,00000001);
            
            if (d->decodeBitCnt < LTC_FRAME_BIT_COUNT) {
                // Isolating the lowest three bits: the location of this bit in the current byte
                bitNum = (d->decodeBitCnt & B8(00000111));
                // Using the bit number to define which of the eight bits to set
                bitSet = (B8(00000001) << bitNum);
                // Isolating the higher bits: the number of the byte/char the target bit is contained in
                bytePos = d->decodeBitCnt >> 3;
                
                (((unsigned char*)&d->decodeFrame)[bytePos]) |= bitSet;
            }
            
        }
        d->decodeBitCnt++;
        
        if (d->decodeSyncWord == B16(00111111,11111101) /*LTC Sync Word 0x3ffd*/) {
            if (d->decodeBitCnt == LTC_FRAME_BIT_COUNT) {
                int frame = (int)frame_to_vf(&d->decodeFrame, &d->fps);
                
                if ( (d->lastFrame == 0) && (d->firstFrame == 0) ) {
                    d->firstFrame = frame;
                    d->lastFrame = frame;
                } else if ( (frame - d->lastFrame) != 1 ) {
                    d->errorCnt++;
                }
                
                d->lastFrame = frame;
                
                memcpy( &d->queue[d->qWritePos].base,
                       &d->decodeFrame,
                       sizeof(SMPTEFrame));
                
                if (d->correctJitter)
                    d->queue[d->qWritePos].delayed = size-i;
                else
                    d->queue[d->qWritePos].delayed = 0;
                
                // TODO: compare: sampleRate*80*soundToBiphasePeriod*delayed
                d->queue[d->qWritePos].startpos = d->decodeFrameStartPos;
                d->queue[d->qWritePos].endpos = posinfo + offset[i];
                
                d->qWritePos++;
                
                if (d->qWritePos == d->qLen)
                    d->qWritePos = 0;
            }
            d->decodeBitCnt = 0;
        }
    }
    return 1;
}

long int ofxLtcReader::frame_to_vf(SMPTEFrame *f, FrameRate *fps) {
    int hours =	(f->hoursUnits + f->hoursTens*10);
    int minutes = (f->minsUnits + f->minsTens*10);
    int seconds = (f->secsUnits + f->secsTens*10);
    int frames = (f->frameUnits + f->frameTens*10);
    
    return FR_smpte2vf(fps, frames, seconds, minutes, hours, 0);
}

long int ofxLtcReader::FR_smpte2vf (FrameRate *fr, int f, int s, int m, int h, int overflow) {
    long int rv = 0 ;
    double fps = (double)fr->num/(double)fr->den; // ought to equal (30.0*1000.0/1001.0)
    
    if (fr->flags&FRF_DROP_FRAMES)
        rv = FR_drop_frames(fr,f,s,m,h);
    else
        rv = f + fps * ( s + 60*m + 3600*h);
    
    return(rv);
}

long int ofxLtcReader::FR_drop_frames (FrameRate *fr, int f, int s, int m, int h) {
    double fps = (double)fr->num/(double)fr->den; // (30.0*1000.0/1001.0)
    int fpsi = (int) rint(fps); // 30.0 !
    long int base_time = ((h*3600) + ((m/10) * 10 * 60)) * fps;
    long off_m = m % 10;
    long off_s = (off_m * 60) + s;
    long off_f = (fpsi * off_s) + f - (2 * off_m);
    return (base_time + (long int) off_f);
}

int ofxLtcReader::SMPTEDecoderWrite(SMPTEDecoder *d, sample_t *buf, int size, long int posinfo) {
    // The offset values below mark the last sample belonging to the respective value.
    unsigned char code[1024]; // decoded biphase values, only one bit per byte is used // TODO: make boolean 1 bit
    unsigned long offs[1024]; // positions in the sample buffer (buf) where each of the values in code was detected
    unsigned char bits[1024]; // bits decoded from code, only one bit per byte is used // TODO: make boolean 1 bit
    unsigned long offb[1024]; // positions in the sample buffer (buf) where each of the values in bits was detected
    
    //TODO check if size <= 1024; dynamic alloc buffers in Decoder struct.
    
    
    
    size = audio_to_biphase(d, buf, code, offs, size);
    
    WRITE_DECODER_BIPHASE_DIAGNOSTICS(d, code, offs, size, posinfo);
    size = biphase_decode(d, code, offs, bits, offb, size);
    WRITE_DECODER_BITS_DIAGNOSTICS(d, bits, offb, size, posinfo);
    return ltc_decode(d, bits, offb, posinfo, size);
}

int ofxLtcReader::SMPTEDecoderRead(SMPTEDecoder* decoder, SMPTEFrameExt* frame) {
    if (!frame) return 0;
    if (decoder->qReadPos != decoder->qWritePos) {
        memcpy(frame, &decoder->queue[decoder->qReadPos], sizeof(SMPTEFrameExt));
        decoder->qReadPos++;
        if (decoder->qReadPos == decoder->qLen)
            decoder->qReadPos = 0;
        return 1;
    }
    return 0;
}

ofxLtcReader::SMPTEDecoder* ofxLtcReader::SMPTEDecoderCreate(int sampleRate, FrameRate *fps, int queueLen, int correctJitter) {
    SMPTEDecoder* d = (SMPTEDecoder*) calloc(1, sizeof(SMPTEDecoder));
    d->sampleRate = sampleRate;
    d->fps.num = fps->num;
    d->fps.den = fps->den;
    d->fps.flags = fps->flags;
    d->qLen = queueLen;
    d->queue = (SMPTEFrameExt*) calloc(d->qLen, sizeof(SMPTEFrameExt));
    d->biphaseToBinaryState = 1;
    d->soundToBiphasePeriod = d->sampleRate / (int)rint(((double)(&(d->fps))->num/(double)(&(d->fps))->den)) / 80;
#ifdef SAMPLE_IS_UNSIGNED
    d->soundToBiphaseLimit = (d->soundToBiphasePeriod * 14) >> 4;
#else
    d->soundToBiphaseLimit = (d->soundToBiphasePeriod * 14) / 16;
#endif
    d->correctJitter = correctJitter;
    
    d->samplesToSeconds = 1 / ((timeu)d->sampleRate * sizeof(sample_t));
    
#ifdef DIAGNOSTICS_OUTPUT
    d->diagnosticsPos = 0;
    d->diagnosticsFile = fopen(DIAGNOSTICS_FILEPATH, "w");
    d->diagnosticsBiphaseFile = fopen(DIAGNOSTICS_FILEPATH "-biphase.txt", "w");
    d->diagnosticsBitsFile = fopen(DIAGNOSTICS_FILEPATH "-bits.txt", "w");
#endif
    
    return d;
}

ofxLtcReader::FrameRate * ofxLtcReader::FR_create(int num, int den, int flags) {
    FrameRate *f = new FrameRate;
    assert(num>0 && den >0 && flags <= FRF_LAST);
    f->num = num;
    f->den = den;
    f->samplerate = 0;
    f->aoffset = 0;
    f->voffset = 0;
    f->flags = flags&~FRF_SAMPLERATE;
    //	f->flags = flags&~(FRF_SAMPLERATE|FRF_AOFFSET|FRF_VOFFSET);
    return(f);
}

void ofxLtcReader::FR_setdbl(FrameRate *fr, double fps, int mode) {
    FR_setratio(fr, rint(fps*100000.0),100000); // FIXME use libgmp - see below
    if (mode && (rint(100.0*fps) == 2997.0)) {
        fr->flags |= FRF_DROP_FRAMES;
        FR_setratio(fr, 30000,1001);
    } else if (mode)
        fr->flags &= ~FRF_DROP_FRAMES;
}

void ofxLtcReader::FR_setratio(FrameRate *fr, int num, int den) {
    fr->num = num;
    fr->den = den;
}

void ofxLtcReader::FR_free(FrameRate *f) {
    if (f) free(f);
}

int ofxLtcReader::SMPTEFrameToTime(SMPTEFrame* frame, SMPTETime* stime) {
#ifdef ENABLE_DATE
    // FIXME: what role does the MJD flag play?
    SMPTESetTimeZoneString(frame, stime);
    
    stime->years = frame->user5 + frame->user6*10;
    stime->months = frame->user3 + frame->user4*10;
    stime->days = frame->user1 + frame->user2*10;
#endif
    stime->hours = frame->hoursUnits + frame->hoursTens*10;
    stime->mins = frame->minsUnits + frame->minsTens*10;
    stime->secs = frame->secsUnits + frame->secsTens*10;
    stime->frame = frame->frameUnits + frame->frameTens*10;
    return 1;
}

int ofxLtcReader::SMPTEDecoderFrameToMillisecs(SMPTEDecoder* d, SMPTEFrameExt* frame, int* timems) {
    int ms = 0;
    
    long int frame_count = frame_to_vf(&frame->base, &d->fps);
    ms = (int)((double)(1000*frame_count) / (double)((&d->fps)->num)/(double)((&d->fps)->den));
    
    *timems = ms;
    if (d->correctJitter) {
        *timems += 1000*frame->delayed/d->sampleRate;
    }
    return 1;	
}

int ofxLtcReader::SMPTEFreeDecoder(SMPTEDecoder *d) {
    if (!d) return 1;
    if (d->queue) free(d->queue);
#ifdef DIAGNOSTICS_OUTPUT
    if (d->diagnosticsFile) fclose(d->diagnosticsFile);
    if (d->diagnosticsBiphaseFile) fclose(d->diagnosticsBiphaseFile);
    if (d->diagnosticsBitsFile) fclose(d->diagnosticsBitsFile);
#endif
    free(d);
    
    return 0;
}

