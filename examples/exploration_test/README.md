## Exploratioin Test

Test code of Exploration Module in C++ embedding.

### Dependencies

Refer to [requirements.txt](https://github.com/deepguider/DeepGuider/blob/master/src/exploration/requirements.txt) in exploration module

### How to Build and Run Codes

1. Run the following shell script:
```
$ ./build_run_exp.sh
```
2. Refer to main.cpp for the usage of the exploration module
```
// Initialize the Python interpreter
init_python_environment("python3", nullptr, false);

// Initialize Python module
ActiveNavigation active_nav;
active_nav.initialize();

// Run the Python module
active_nav.apply(image, guidance, t);
std::vector<ExplorationGuidance> actions;
GuidanceManager::GuideStatus status;
active_nav.get(actions, status);

// Clear the Python module
active_nav.clear();

// Close the Python Interpreter
close_python_environment();
```
