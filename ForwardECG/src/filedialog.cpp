#include "filedialog.h"
#include <Windows.h>


std::string open_file_dialog(const std::string& initial_name, const std::string& filter)
{
	OPENFILENAMEA ofn;
	TCHAR szFile[260*4] = { 0 };

	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = filter.c_str();
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = (char*)initial_name.c_str();
	ofn.nMaxFileTitle = initial_name.size();
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	if (GetOpenFileNameA(&ofn) == TRUE)
	{
		return szFile;
	}

	return "";
}

std::string save_file_dialog(const std::string& initial_name, const std::string& filter)
{
	OPENFILENAMEA ofn;
	TCHAR szFile[260*4] = { 0 };

	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = filter.c_str();
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = (char*)initial_name.c_str();
	ofn.nMaxFileTitle = initial_name.size();
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = 0;

	if (GetSaveFileNameA(&ofn) == TRUE)
	{
		return szFile;
	}

	return "";
}
