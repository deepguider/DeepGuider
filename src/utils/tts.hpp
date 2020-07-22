/***
 * ccsmm@etri.re.kr
 * You need to install follows in ubuntu(default is python 2.7).:
 *		pip install gTTS
 *		sudo apt install ffmpeg 
***/

#ifndef __DG_TTS__
#define __DG_TTS__

#include <bits/stdc++.h> 

namespace dg
{

int tts(std::string sentence)
{
	// gtts-cli '안녕하세요' --output tmp.mp3;ffplay -nodisp -autoexit tmp.mp3 >/dev/null 2>&1
	std::string str = "gtts-cli '";
	str = str + sentence + "' --output tmp.mp3;ffplay -nodisp -autoexit tmp.mp3 >/dev/null 2>&1";
	const char *command = str.c_str(); 
	system(command);
	return 0;
}

} // End of 'dg'

#endif // End of '__DG_TTS__'
