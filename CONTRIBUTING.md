## How to Contribute DeepGuider
_DeepGuider_ aims for the cross-platform development across operating systems (Windows, Linux) and programming languages (C++, Python). Its main source codes are maintained in [its Github repository](https://github.com/deepguider/DeepGuider) and its API documentation is available on [its Github IO](https://deepguider.github.io/).

You can contibute DeepGuider as

* (As a member) Pushing  your modules or  modification directly to [the repository](https://github.com/deepguider/DeepGuider)
* (As a non-member) Sending your [pull request](https://help.github.com/en/articles/creating-a-pull-request) after modifying codes
* Posting any question, bug report, and feature request on [the issue tracker](https://github.com/deepguider/DeepGuider/issues)

### C/C++ Guideline
* Namespace: `dg`
* Coding and naming convention: [The OpenCV Coding Syle Guide](https://github.com/opencv/opencv/wiki/Coding_Style_Guide)
  * Indentation: **4 spaces**
  * File naming: **snake_case** (e.g. `basic_type.hpp`)
  * File extension: `hpp` for headers and `cpp` for implementation
    * A header file starts from code fence as follows.
    ```cpp
    #ifndef __HEADER_NAME__
    #define __HEADER_NAME__
    namespace dg
    {
    ...
    }
    #endif // End of '__HEADER_NAME__'  
    ```
  * Class naming: **PascalCase** (e.g. `RoadDetector`)
  * Function naming: **camelCase** (e.g. `addRoad`)
    * Function names usually start from _verb_ or its abbreviation. (e.g. `calibrateCamera`, `calcOpticalFlowPyrLK`)
  * Variable naming: **snake_case** (e.g. `road_map`)
    * Member variables are starting with prefix `m_`. (e.g. `m_road_map`)
* Comments for API documentation: [Doxygen](http://www.doxygen.nl/) (see [directed_graph.hpp](https://github.com/deepguider/RoadGPS/blob/master/src/core/directed_graph.hpp))
* Directory convention
  *  `bin`: Include data for simple tests and DLL and executables for Windows
  *  `doc`: Include `Doxyfile.in` and supplementary images for documentation
  *  `examples`: Include examples with DeepGuider (e.g. `unit_test`)
  *  `EXTERNAL`: Include external source codes
  *  `src`: Include source codes of DeepGuider modules

### Python Guideline
TODO

### Tips
* How to execute examples
  * Please refer [README.md](README.md).
* How to make your own example in MSVS
  * Just copy `examples/proj_template` and changing names to your own name.