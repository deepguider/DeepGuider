/**
 * This file is a part of OpenCX, especially about functionalities not depending on OpenCV.
 * - Homepage: https://github.com/sunglok/opencx
 */

/**
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <sunglok@hanmail.net> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Sunglok Choi
 * ----------------------------------------------------------------------------
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
        String2D extString2D(size_t row_start = 0, const std::vector<size_t> columns = std::vector<size_t>(), const std::string& invalid_val = "None")
        {
            String2D data;
            if (!this->empty())
            {
                // Select all columns if the given is empty
                std::vector<size_t> col_select = columns;
                if (col_select.empty()) col_select = getColumns(this->front().size());

                // Extract the selected columns
                for (size_t row = row_start; row < this->size(); row++)
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
        Double2D extDouble2D(size_t row_start = 0, const std::vector<size_t> columns = std::vector<size_t>(), double invalid_val = std::numeric_limits<double>::quiet_NaN())
        {
            Double2D data;
            if (!this->empty())
            {
                // Select all columns if the given is empty
                std::vector<size_t> col_select = columns;
                if (col_select.empty()) col_select = getColumns(this->front().size());

                // Extract the selected columns
                for (size_t row = row_start; row < this->size(); row++)
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
        Int2D extInt2D(size_t row_start = 0, const std::vector<size_t> columns = std::vector<size_t>(), int invalid_val = -1)
        {
            Int2D data;
            if (!this->empty())
            {
                // Select all columns if the given is empty
                std::vector<size_t> col_select = columns;
                if (col_select.empty()) col_select = getColumns(this->front().size());

                // Extract the selected columns
                for (size_t row = row_start; row < this->size(); row++)
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
        static std::vector<size_t> getColumns(size_t size, size_t start = 0, size_t step = 1)
        {
            std::vector<size_t> permutation(size);
            for (size_t i = 0; i < permutation.size(); i++)
                permutation[i] = start + step * i;
            return permutation;
        }
    }; // End of 'CSVReader'

    /**
     * @brief The range of numeric values
     *
     * The range of numeric values is described in its lower and upper values.
     */
    template <typename T>
    class Range_
    {
    public:
        /**
         * A constructor with initialization
         * @param _min The lower bound of the given range
         * @param _min The upper bound of the given range
         */
        Range_(T _min = 0, T _max = 0)
        {
            if (_min < _max)
            {
                min = _min;
                max = _max;
            }
            else
            {
                min = _max;
                max = _min;
            }
        }

        /**
         * Get the length of the range
         * @return The length of the range
         */
        T length() const { return max - min; }

        /**
         * Get the middle value of the range
         * @return The middle value of the range
         */
        double center() const { return (max + min) / 2.; }

        /** The lower bound of the range */
        T min;

        /** The upper bound of the range */
        T max;
    };

    /** The predefined range for the integer type */
    typedef Range_<int> Range;

    /** The predefined range for the integer type */
    typedef Range_<int> RangeInt;

    /** The predefined range for the double (a.k.a. float64) type */
    typedef Range_<double> RangeDbl;

} // End of 'cx'

#endif // End of '__OPEN_SX__'
