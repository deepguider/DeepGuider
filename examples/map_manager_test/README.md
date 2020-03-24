

## MapManager

Map management modules to interact with cloud maps and paths

### Dependency
* [RapidJSON](https://rapidjson.org/) ([License](https://github.com/Tencent/rapidjson/blob/master/license.txt))
  * _RapidJSON_ is a fast JSON parser/generator for C++ with both SAX/DOM style API and header-only C++ library.
  * It is included in `EXTERNAL` directory.
* [libcurl](https://curl.haxx.se/libcurl/) ([License](https://curl.haxx.se/docs/copyright.html))
  * _libcurl_ is a free and easy-to-use client-side URL transfer library, supporting DICT, FILE, FTP, FTPS, Gopher, HTTP, HTTPS, IMAP, IMAPS, LDAP, LDAPS, POP3, POP3S, RTMP, RTSP, SCP, SFTP, SMTP, SMTPS, Telnet and TFTP.
  * Download and Install
    * [Windows](https://curl.haxx.se/download.html)
    * **Linux**
      ```
      $ sudo apt-get install libcurl4-openssl-dev
      ```
  * It is included in `EXTERNAL` directory.
* [QGC Ground Control Station](https://github.com/mavlink/qgroundcontrol) ([Apache 2.0 License](https://github.com/mavlink/qgroundcontrol/blob/master/COPYING.md))
  * UTM-related routines in _QGC Ground Control Station_ is utilized for conversion between geodesic notation (latitude and longitude) and UTM.
  * It is included in `EXTERNAL` directory.


### How to Build and Run Codes
```
$ ./build_run_mmt.sh
```