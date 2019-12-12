#include <vector>
#include <windows.h>
#include <SetupAPI.h>

typedef std::vector<std::pair<UINT, std::string> > CPortAndNamesArray;

bool QueryUsingSetupAPI(const GUID& guid, _In_ DWORD dwFlags, _Inout_ CPortAndNamesArray& ports);
