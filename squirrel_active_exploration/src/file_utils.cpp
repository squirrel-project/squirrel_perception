#include "squirrel_active_exploration/file_utils.h"

using namespace std;

void replace_substr(string& str, const string& from, const string& to)
{
    size_t start_pos = str.find(from);
    if (start_pos != string::npos)
    {
        for (size_t pos = 0; ; pos += to.length())
        {
            // Locate the substring to replace
            pos = str.find(from, pos);
            if (pos == string::npos)
                break;
            // Replace by erasing and inserting
            str.erase (pos, from.length());
            str.insert (pos, to);
        }
    }

}

string add_backslash(const string &str)
{
    string res = str;
    // If the string is empty
    if (str.size() == 0)
    {
        res = "/";
        return res;
    }
    // If the last character is already a back slash then ignore
    char last_char = str[str.length()-1];
    if (last_char != '/')
        res = str + "/";
    return res;
}

string rem_backslash(const string &str)
{
    string res = str;
    // If the string is empty
    if (str.size() == 0)
    {
        res = "";
        return res;
    }
    // If the last character is not a back slash then ignore
    char last_char = str[str.length()-1];
    if (last_char == '/')
        res = str.substr(0,str.length()-1);
    return res;
}

bool split_filename(const string& str, string &path, string &file)
{
    // Find the last directory delimiter
    size_t found = str.find_last_of("/\\");
    // If did not find the directory delimiter
    if (found == string::npos)
        return false;
    // Split to path and filename
    path = str.substr(0,found);
    file = str.substr(found+1);
    // Return success
    return true;
}
