#ifndef SPEAKER_H
#define SPEAKER_H
#include <Arduino.h>
#include "SpeakerPitches.h"
#include "PinDef.h"


// Referenced: https://docs.arduino.cc/built-in-examples/digital/toneMelody/
// Durations set per melody note
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int start_duration[] = {
    2, 2, 2, 1
};

// These are the notes that the speaker will play
int start_melody[] = {
    NOTE_A3, NOTE_A3, NOTE_A3, NOTE_A5
};

int object_duration[] = {
    8, 8, 8
};

int object_melody[] = {
    NOTE_A3, NOTE_A3, NOTE_A3
};

int stop_duration[] = {
    8, 8

};

int stop_melody[] = {
    NOTE_C8, NOTE_C8
};

int readystart_duration[] = {
    8, 4
};

int readystart_melody[] = {
    NOTE_C4, NOTE_A4
};

int unknown_melody[] = {
    NOTE_A3, NOTE_A6, NOTE_C4
};

int unknown_duration[] = {
    8, 8, 4
};

//Speaker Type ENUMS
enum speakerHeaders{
    OBJECT_DETECTED = -1,
    UNKNOWN_ERROR = 4,
    STOP_TONE = 1, 
    START_TONE = 2,
    READYTOSTART_TONE = 3
};

// Play tones based on header given
void play_tone(speakerHeaders header){
    switch(header){
        case OBJECT_DETECTED:
            play_tone_helper(object_duration, object_melody);
        case STOP_TONE:
            play_tone_helper(stop_duration, stop_melody);
        case START_TONE:
            play_tone_helper(start_duration, start_melody);
        case READYTOSTART_TONE:
            play_tone_helper(readystart_duration, readystart_melody);
        default:
            play_tone_helper(unknown_duration, unknown_melody);
    }
}


// Play different tones based on input of melody and duration int arrays
void play_tone_helper(int duration[], int melody[]){
    for (int thisNote = 0; thisNote < 8; thisNote++) {
        // to calculate the note duration, take one second divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / duration[thisNote];
        tone(10, melody[thisNote], noteDuration);
        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 1.75;
        delay(pauseBetweenNotes);
        // stop the tone playing:
        noTone(speaker_pin);
    }
}


#endif