/***
 * ccsmm@etri.re.kr
 * You need to install follows in ubuntu(default is python 2.7).:
 *		pip install gTTS
 *		sudo apt install ffmpeg 
***/

#ifndef __DG_TTS__
#define __DG_TTS__

#ifndef _WIN32
#include <bits/stdc++.h> 
#endif	// _WIN32

namespace dg
{

int tts(std::string sentence, bool print_msg = true)
{
	if(print_msg)
	{
		printf("[TTS] %s\n", sentence.c_str());
	}

#ifndef _WIN32
	// gtts-cli '안녕하세요' --output tmp.mp3;ffplay -nodisp -autoexit tmp.mp3 >/dev/null 2>&1
	std::string str = "gtts-cli '";
	str = str + sentence + "' --output tmp.mp3;ffplay -nodisp -autoexit tmp.mp3 >/dev/null 2>&1";
	const char *command = str.c_str(); 
	system(command);
#endif	// _WIN32
	return 0;
}

} // End of 'dg'

#endif // End of '__DG_TTS__'
