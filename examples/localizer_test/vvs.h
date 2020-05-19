/**
 * VVS: A Very Very Simple Unit Testing Tool for C/C++
 *
 * VVS aims to provide a very small and simple framework for unit testing in C and C++.
 * It consists of a single header file, 'vvs.h', which only utilize C standard libraries.
 * Just include the file to your project. It will work without complex configuration and dependency.
 * VVS is Beerware so that it is free to use and distribute.
 *
 * - Homepage: https://github.com/sunglok/vvs
 *
 * @author  Sunglok Choi (http://sites.google.com/site/sunglok)
 * @version 0.2 (05/22/2019)
 */

/**
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <sunglok@hanmail.net> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Sunglok Choi
 * ----------------------------------------------------------------------------
 */

#ifndef __VVS_UNIT_TEST__
#define __VVS_UNIT_TEST__

#include <stdio.h>
#include <assert.h>
#include <time.h>

/**
 * An output stream to display test results
 * @see stdout, stderr
 */
#define VVS_OUTPUT                      (stdout)

/**
 * A constant to identify similarity of real values
 * @see VVS_TEST_CLOSE
 */
#define VVS_EPSILON                     (1e-6)

/**
 * A text to notify the given test is successful
 */
#define VVS_SUCCESS_MSG                 (" --> Success!\n")

/**
 * A text to notify the given test is failed
 */
#define VVS_FAILURE_MSG                 (" --> Failure!\n")

/**
 * A text (with two integer values) to notify the given test is failed
 */
#define VVS_FAILURE_MSG1                (" --> Failure! (%d, %d)\n")

/**
 * A text (with two real values) to notify the given test is failed
 */
#define VVS_FAILURE_MSG2                (" --> Failure! (%f, %f)\n")

/**
 * A text format to show location of the given expression <p>
 * If you don't want to display it, please make it an empty macro.
 */
#define VVS_LOCATION_MSG                (" --> File: %s / Line: %d\n")

/**
 * A text format to show elapsed time <p>
 * If you don't want to display it, please make it an empty macro.
 */
#define VVS_TIME_MSG                    (" --> Time: %.6f [sec]\n")

/**
 * A function for assert <p>
 * If you don't want to use it, please make it an empty macro.
 */
#define VVS_ASSERT                      assert

/**
 * Verify that the given expression is true
 * @param EXP the given expression
 */
#define VVS_CHECK_TRUE(EXP) \
    { \
        int _isTrue_ = (int)(EXP); \
        fprintf(VVS_OUTPUT, "[CHECK_TRUE] " #EXP); \
        if (_isTrue_) fprintf(VVS_OUTPUT, VVS_SUCCESS_MSG); \
        else \
        { \
            fprintf(VVS_OUTPUT, VVS_FAILURE_MSG); \
            fprintf(VVS_OUTPUT, VVS_LOCATION_MSG, __FILE__, __LINE__); \
        } \
        VVS_ASSERT(_isTrue_); \
    }

/**
 * Verify that the given expression is false
 * @param EXP the given expression
 */
#define VVS_CHECK_FALSE(EXP) \
    { \
        int _isTrue_ = (int)(EXP); \
        fprintf(VVS_OUTPUT, "[CHECK_FALSE] " #EXP); \
        if (!_isTrue_) fprintf(VVS_OUTPUT, VVS_SUCCESS_MSG); \
        else \
        { \
            fprintf(VVS_OUTPUT, VVS_FAILURE_MSG); \
            fprintf(VVS_OUTPUT, VVS_LOCATION_MSG, __FILE__, __LINE__); \
        } \
        VVS_ASSERT(!_isTrue_); \
    }

/**
 * Verify that the given two integer values are equal
 * @param VAL1 the first integer value
 * @param VAL2 the second integer value
 */
#define VVS_CHECK_EQUAL(VAL1, VAL2) \
    { \
        int _val1_ = (int)(VAL1); \
        int _val2_ = (int)(VAL2); \
        int _isEqual_ = _val1_ == _val2_; \
        fprintf(VVS_OUTPUT, "[CHECK_EQUL] " #VAL1 " == " #VAL2); \
        if (_isEqual_) fprintf(VVS_OUTPUT, VVS_SUCCESS_MSG); \
        else \
        { \
            fprintf(VVS_OUTPUT, VVS_FAILURE_MSG1, _val1_, _val2_); \
            fprintf(VVS_OUTPUT, VVS_LOCATION_MSG, __FILE__, __LINE__); \
        } \
        VVS_ASSERT(_isEqual_); \
    }

/**
 * Verify that the given two real values are near
 * @param VAL1 the first real value
 * @param VAL2 the second real value
 * @param EPS the threshold of difference
 */
#define VVS_CHECK_RANGE(VAL1, VAL2, EPS) \
    { \
        double _val1_ = (double)(VAL1); \
        double _val2_ = (double)(VAL2); \
        double _delta_ = _val1_ - _val2_; \
        int _isNear_ = (-EPS < _delta_) && (_delta_ < +EPS); \
        fprintf(VVS_OUTPUT, "[CHECK_NEAR] " #VAL1 " == " #VAL2); \
        if (_isNear_) fprintf(VVS_OUTPUT, VVS_SUCCESS_MSG); \
        else \
        { \
            fprintf(VVS_OUTPUT, VVS_FAILURE_MSG2, _val1_, _val2_); \
            fprintf(VVS_OUTPUT, VVS_LOCATION_MSG, __FILE__, __LINE__); \
        } \
        VVS_ASSERT(_isNear_); \
    }

/**
 * Verify that the given two integer values are equal
 * @param VAL1 the first integer value
 * @param VAL2 the second integer value
 */
#define VVS_CHECK_EQUL(VAL1, VAL2)      VVS_CHECK_EQUAL(VAL1, VAL2)

/**
 * Verify that the given two real values are near
 * @param VAL1 the first real value
 * @param VAL2 the second real value
 */
#define VVS_CHECK_NEAR(VAL1, VAL2)      VVS_CHECK_RANGE(VAL1, VAL2, VVS_EPSILON)

/**
 * Run the given expression or function (a set of tests)
 * @param EXP the given expression or function
 */
#define VVS_RUN_TEST(EXP) \
    { \
        clock_t _t; \
        fprintf(VVS_OUTPUT, "=== " #EXP " ===" "\n"); \
        _t = clock(); \
        (EXP); \
        _t = clock() - _t; \
        fprintf(VVS_OUTPUT, VVS_TIME_MSG, ((double)_t) / CLOCKS_PER_SEC); \
    } \
     
/**
 * Do not run the given expression <p>
 * It is useful to disable RUN_UNIT_TEST instead of applying comment.
 * @param EXP the given expression or function
 */
#define VVS_NUN_TEST(EXP)               { }

#endif // End of '__VVS_UNIT_TEST__'
