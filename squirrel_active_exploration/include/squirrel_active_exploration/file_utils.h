#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <stdlib.h>
#include <string>

void replace_substr(std::string& str, const std::string& from, const std::string& to);

std::string add_backslash(const std::string &str);

std::string rem_backslash(const std::string &str);

bool split_filename(const std::string& str, std::string &path, std::string &file);

#endif // FILE_UTILS_H
