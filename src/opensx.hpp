/**
 * OpenCX: Sunglok's OpenCV Extension
 *
 * OpenCX aims to provide extended functionality and tools to OpenCV for more convenience.
 * It consists of a single header file, 'opencx.hpp', which only depends on OpenCV in C++.
 * Just include the file to your project. It will work without complex configuration and dependency.
 * OpenCX is Beerware so that it is free to use and distribute.
 *
 * - Homepage: https://github.com/sunglok/opencx
 *
 * @author  Sunglok Choi (http://sites.google.com/site/sunglok)
 * @version 0.3 (12/10/2019)
 */

/**
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <sunglok@hanmail.net> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Sunglok Choi
 * ----------------------------------------------------------------------------
 */

/**
 * This file is a part of OpenCX, but it contains functionalities which does not depend on OpenCV.
 */

#ifndef __OPEN_SX__
#define __OPEN_SX__

#include <string>
#include <functional>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>

namespace cx
{
    /**
     * Remove space at the left of the given string
     * @param text The given string
     * @return The trimmed string
     */
    inline std::string trimLeft(std::string text)
    {
        // Reference: https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
        text.erase(text.begin(), std::find_if(text.begin(), text.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return text;
    }

    /**
     * Remove space at the right of the given string
     * @param text The given string
     * @return The trimmed string
     */
    inline std::string trimRight(std::string text)
    {
        text.erase(std::find_if(text.rbegin(), text.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), text.end());
        return text;
    }

    /**
     * Remove space at the left and right of the given string
     * @param text The given string
     * @return The trimmed string
     */
    inline std::string trimBoth(std::string text) { return trimLeft(trimRight(text)); }

    /**
     * Make the given string to its lower cases
     * @param text The given string
     * @return The transformed string with lower cases
     */
    inline std::string toLowerCase(std::string text)
    {
        // Reference: https://stackoverflow.com/questions/313970/how-to-convert-stdstring-to-lower-case
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
        return text;
    }

    /**
     * @brief A CSV File Reader
     *
     * A CSV file is a popular text file format for reading and writing tubular data.
     * It is also supported by many other applications such as Microsoft Excel and Open Office.
     * This is an extension of a 2D vector of string with additional functions for reading a CSV file and extracting its columns as a 2D vector of doubles (a.k.a. float64) or integers (a.k.a. int32) or strings.
     */
    class CSVReader : public std::vector<std::vector<std::string>>
    {
    public:
        /** A type for 2D vector of strings */
        typedef CSVReader String2D;

        /** A type for 2D vector of doubles */
        typedef std::vector<std::vector<double>> Double2D;

        /** A type for 2D vector of integers */
        typedef std::vector<std::vector<int>> Int2D;

        /**
         * Open a CSV file
         * @param csv_file The filename to read
         * @param separator A character which separate each column
         * @return True if successful (false if failed)
         */
        bool open(const std::string& csv_file, char separator = ',')
        {
            std::ifstream file(csv_file);
            if (!file.is_open()) return false;

            this->clear();
            std::string line;
            while (getline(file, line))
            {
                std::vector<std::string> datum;
                std::string element;
                std::stringstream temp(line);
                while (getline(temp, element, separator))
                    datum.push_back(trimBoth(element));
                this->push_back(datum);
            }
            return true;
        }

        /**
         * Extract the desired columns as a 2D vector of strings
         * @param row_start A starting row to skip headers
         * @param columns The desired columns to extract
         * @param invalid_val The value to represent invalid (e.g. non-numeric) elements
         * @return The selected columns as a 2D vector of strings
         */
        String2D extString2D(int row_start = 0, const std::vector<int> columns = std::vector<int>(), const std::string& invalid_val = "None")
        {
            String2D data;
            if (!this->empty())
            {
                // Select all columns if the given is empty
                std::vector<int> col_select = columns;
                if (col_select.empty()) col_select = getColumns(this->front().size());

                // Extract the selected columns
                for (auto row = row_start; row < this->size(); row++)
                {
                    const std::vector<std::string>& row_data = this->at(row);
                    if (row_data.empty()) continue;
                    std::vector<std::string> vals;
                    for (auto col = col_select.begin(); col != col_select.end(); col++)
                    {
                        if (*col < row_data.size()) vals.push_back(row_data[*col]);
                        else vals.push_back(invalid_val);
                    }
                    data.push_back(vals);
                }
            }
            return data;
        }

        /**
         * Extract the desired columns as a 2D vector of doubles (a.k.a. float64)
         * @param row_start A starting row to skip headers
         * @param columns The desired columns to extract
         * @param invalid_val The value to represent invalid (e.g. non-numeric) elements
         * @return The selected columns as a 2D vector of doubles
         */
        Double2D extDouble2D(int row_start = 0, const std::vector<int> columns = std::vector<int>(), double invalid_val = std::numeric_limits<double>::quiet_NaN())
        {
            Double2D data;
            if (!this->empty())
            {
                // Select all columns if the given is empty
                std::vector<int> col_select = columns;
                if (col_select.empty()) col_select = getColumns(this->front().size());

                // Extract the selected columns
                for (auto row = row_start; row < this->size(); row++)
                {
                    const std::vector<std::string>& row_data = this->at(row);
                    if (row_data.empty()) continue;
                    std::vector<double> vals;
                    for (auto col = col_select.begin(); col != col_select.end(); col++)
                    {
                        double val = invalid_val;
                        try
                        {
                            if (*col < row_data.size())
                                val = std::stod(row_data[*col]);
                        }
                        catch (std::exception e) { }
                        vals.push_back(val);
                    }
                    data.push_back(vals);
                }
            }
            return data;
        }

        /**
         * Extract the desired columns as a 2D vector of integers (a.k.a. int32)
         * @param row_start A starting row to skip headers
         * @param columns The desired columns to extract
         * @param invalid_val The value to represent invalid (e.g. non-numeric) elements
         * @return The selected columns as a 2D vector of integers
         */
        Int2D extInt2D(int row_start = 0, const std::vector<int> columns = std::vector<int>(), int invalid_val = -1)
        {
            Int2D data;
            if (!this->empty())
            {
                // Select all columns if the given is empty
                std::vector<int> col_select = columns;
                if (col_select.empty()) col_select = getColumns(this->front().size());

                // Extract the selected columns
                for (auto row = row_start; row < this->size(); row++)
                {
                    const std::vector<std::string>& row_data = this->at(row);
                    if (row_data.empty()) continue;
                    std::vector<int> vals;
                    for (auto col = col_select.begin(); col != col_select.end(); col++)
                    {
                        int val = invalid_val;
                        try
                        {
                            if (*col < row_data.size())
                                val = std::stoi(row_data[*col]);
                        }
                        catch (std::exception e) { }
                        vals.push_back(val);
                    }
                    data.push_back(vals);
                }
            }
            return data;
        }

        /**
         * Get a permutation of the given length, starting value, and step
         * @param size The length of the permutation
         * @param start The given starting value
         * @param step The given step
         * @return A series of integers
         */
        static std::vector<int> getColumns(size_t size, int start = 0, int step = 1)
        {
            std::vector<int> permutation(size);
            std::fill(permutation.begin(), permutation.end(), start += step);
            return permutation;
        }

    }; // End of 'CSVReader'
} // End of 'cx'

#endif // End of '__OPEN_SX__'
