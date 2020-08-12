/***
 * ccsmm@etri.re.kr
 * You need to install follows in ubuntu(default is python 2.7).:
 *		pip install gTTS
 *		sudo apt install ffmpeg 
***/

#ifndef __DG_TTS__
#define __DG_TTS__
#define __DG_TTS_USE_SAVED_MP3__

#ifndef _WIN32
#include <bits/stdc++.h> 
#endif	// _WIN32

namespace dg
{

#ifndef _WIN32

/*** Note that each messages(ex, "Go_forward") should include no space.
 *	 And the length of each message should not be too long,
 *	 because itself is used as a file name : [message].mp3"
 ****/

std::string tts_msg_list_example[] = {
	"Go_forward",
	"Cross_forward",
	"Enter_forward",
	"Exit_forward",
	"Turn_left",
	"Cross_left",
	"Enter_left",
	"Exit_left",
	"Turn_right",
	"Cross_right",
	"Enter_right",
	"Exit_right",
	"Turn_back"
};

int _tts_mp3play(string fname_noext) // use ffmpeg to play mp3
{
	int ret = 0;
	std::string cmd_str = "ffplay -nodisp -autoexit ";
	cmd_str += fname_noext +".mp3 >/dev/null 2>&1";
	const char *command = cmd_str.c_str(); 
	ret = system(command);
	if (ret)
	{
		cout << "Error in ffplay : install ffmpeg using \" sudo apt install ffmpeg \", and check " << fname_noext << ".mp3\n";
	}
	return ret;
}

int _tts_mp3gen(string sentence, string fname_noext) // use ffmpeg to play mp3
{
	int ret = 0;
	string cmd_str = "gtts-cli '";
	cmd_str += sentence + "' --output " + fname_noext.c_str() + ".mp3";
	const char *command = cmd_str.c_str(); 
	ret = system(command);
	if (ret)
	{
		cout << "Error in gtts-cli : install gtts \" pip install gTTS \"\n";
	}
	return ret;
}

int _tts(string sentence) // use ffmpeg to play mp3
{
	int ret = 0;
	std::string fname_noext = "fname_tmp";
	ret = _tts_mp3gen(sentence, fname_noext);
	ret = _tts_mp3play(fname_noext);
	return ret;
}



// [For test] Generate all tts mp3 files at offline step
int _tts_mp3gen_test(std::string * tts_msg_list)
{
	int i = 0;
	int ret = 0;
	std::string sentence;
	std::string fname_noext;

	while(!tts_msg_list[i].empty()) 
	{
		sentence = tts_msg_list[i];
		fname_noext = tts_msg_list[i];
		cout << "Generate : " << tts_msg_list[i] << ".mp3\n";

		ret = _tts_mp3gen(tts_msg_list[i], tts_msg_list[i]);
		if (ret > 0) return ret; // It meets error during system() call

		i++;
	}
}


// [For test] Play all tts mp3 files at online step
int _tts_mp3play_test(std::string * tts_msg_list)
{
	int i = 0;
	int ret = 0;
	std::string fname_noext;

	while(!tts_msg_list[i].empty()) 
	{
		fname_noext = tts_msg_list[i];
		cout << "Play : " << fname_noext << ".mp3\n";

		ret = _tts_mp3play(tts_msg_list[i]);
		if (ret > 0) return ret; // It meets error during system() call

		i++;
	}
}

#endif	// _WIN32

int tts(std::string sentence, bool print_msg = true)
{
	int ret = 0;
	if(print_msg)
	{
		printf("[TTS] %s\n", sentence.c_str());
	}

#ifndef _WIN32

#ifdef __DG_TTS_USE_SAVED_MP3__
	ret = _tts_mp3play(sentence); // ret is exit code of system call, non-zero ret means some error.
#else
	ret = _tts(sentence); // ret is exit code of system call, non-zero ret means some error.
#endif //__DG_TTS_USE_SAVED_MP3__

#endif	// _WIN32
	return ret;
}

} // End of 'dg'

#endif // End of '__DG_TTS__'
