#ifndef __DG_UTILS_UTILITY__
#define __DG_UTILS_UTILITY__

#include <vector>
#include <string>

namespace dg
{

std::vector<string> splitStr(const char* buf, int buf_len, char dlm = ',')
{
    int i = 0;
    std::vector<string> vstr;

    // parsing data
    char tmp[256];
    int tmp_len = 256;
    tmp[0] = '\0';
    while (i < buf_len && buf[i] != '\0')
    {
        // skip heading blanks
        while (i < buf_len && buf[i] == ' ') i++;

        int k = 0;
        while (i < buf_len && buf[i] != dlm && buf[i] != '\0' && k < tmp_len)
        {
            tmp[k++] = buf[i++];
        }
        if (k >= tmp_len) break;

        tmp[k] = '\0';
        vstr.push_back(tmp);

        if (buf[i] == '\0') break;
        i++;
    }

    return vstr;
}

} // End of 'dg'

#endif // End of '__DG_UTILS_UTILITY__'
