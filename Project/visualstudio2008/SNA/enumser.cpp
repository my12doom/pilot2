#include "enumser.h"


#pragma comment (lib, "Setupapi.lib")


bool QueryRegistryPortName(HKEY deviceKey, _Out_ int& nPort)
{
  //What will be the return value from the method (assume the worst)
  bool bAdded = false;

  //Read in the name of the port
  char sPortName[1024] = {0};
  DWORD size = sizeof(sPortName)-1;
  DWORD v = RegQueryValueExA(deviceKey, "PortName", 0, NULL, (LPBYTE)sPortName, &size);
  if (v == ERROR_SUCCESS)
  {
    //If it looks like "COMX" then
    //add it to the array which will be returned
    const size_t nLen = strlen(sPortName);
    if (nLen > 3)
    {
      if ((strnicmp(sPortName, "COM", 3) == 0) && atoi(sPortName + 3))
      {
        //Work out the port number
        nPort = atoi(sPortName + 3);
        bAdded = true;
      }
    }
  }

  return bAdded;
}

bool QueryDeviceDescription(_In_ HDEVINFO hDevInfoSet, _In_ SP_DEVINFO_DATA& devInfo, _Inout_ char *sFriendlyName)
{
  DWORD dwType = 0;
  DWORD dwSize = 0;
  //Query initially to get the buffer size required
  if (!SetupDiGetDeviceRegistryProperty(hDevInfoSet, &devInfo, SPDRP_DEVICEDESC, &dwType, NULL, 0, &dwSize))
  {
    if (GetLastError() != ERROR_INSUFFICIENT_BUFFER)
      return false;
  }
#pragma warning(suppress: 26446 26490)
  if (!SetupDiGetDeviceRegistryPropertyA(hDevInfoSet, &devInfo, SPDRP_DEVICEDESC, &dwType, reinterpret_cast<PBYTE>(&(sFriendlyName[0])), dwSize, &dwSize))
    return false;
  if (dwType != REG_SZ)
  {
    SetLastError(ERROR_INVALID_DATA);
    return false;
  }
  return true;
}

bool QueryUsingSetupAPI(const GUID& guid, _In_ DWORD dwFlags, _Inout_ CPortAndNamesArray& ports)
{
  //Set our output parameters to sane defaults
  ports.clear();

  //Create a "device information set" for the specified GUID
  HDEVINFO hDevInfoSet = SetupDiGetClassDevs(&guid, NULL, NULL, dwFlags);
  if (hDevInfoSet == INVALID_HANDLE_VALUE)
    return false;

  //Finally do the enumeration
  bool bMoreItems = true;
  int nIndex = 0;
  SP_DEVINFO_DATA devInfo = { 0 };
  while (bMoreItems)
  {
    //Enumerate the current device
    devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
    bMoreItems = SetupDiEnumDeviceInfo(hDevInfoSet, nIndex, &devInfo);
    if (bMoreItems)
    {
      //Did we find a serial port for this device
      bool bAdded = false;

      std::pair<UINT, std::string> pair;

      //Get the registry key which stores the ports settings
      HKEY deviceKey = SetupDiOpenDevRegKey(hDevInfoSet, &devInfo, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
      if (deviceKey != INVALID_HANDLE_VALUE)
      {
        int nPort = 0;
#pragma warning(suppress: 26486)
        if (QueryRegistryPortName(deviceKey, nPort))
        {
          pair.first = nPort;
          bAdded = true;
        }

		RegCloseKey(deviceKey);
      }

      //If the port was a serial port, then also try to get its friendly name
      if (bAdded)
      {
		  char temp[1024];
        if (QueryDeviceDescription(hDevInfoSet, devInfo, temp))
		{
			pair.second = temp;
			ports.push_back(pair);
		}
      }
    }

    ++nIndex;
  }

  //Free up the "device information set" now that we are finished with it
  SetupDiDestroyDeviceInfoList(hDevInfoSet);

  //Return the success indicator
  return true;
}
