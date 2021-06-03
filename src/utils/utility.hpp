#ifndef __DG_UTILS_UTILITY__
#define __DG_UTILS_UTILITY__

#include <vector>
#include <string>

namespace dg
{

std::vector<std::string> splitStr(const char* buf, int buf_len, char dlm = ',');

} // End of 'dg'

#endif // End of '__DG_UTILS_UTILITY__'
