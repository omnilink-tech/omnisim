// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#include "OmSysInfo.hpp"

#include "OmMacAddress.hpp"

#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>
#include <QtGui/QOpenGLFunctions>

#include <cassert>

#ifdef __linux__
#include <math.h>
#include <unistd.h>
#include <QtCore/QFile>
#endif

#ifdef _WIN32
#include "OmWindowsRegistry.hpp"

#include <cpuid.h>
#include <d3d9.h>
typedef void(WINAPI *PGNSI)(LPSYSTEM_INFO);
#else
#include <sys/utsname.h>
#endif

#ifdef __APPLE__
#include <IOKit/IOKitLib.h>
#include <mach/mach.h>
#include <sys/sysctl.h>
#endif

#ifdef __WIN32
static quint32 gDeviceId = 0;
static quint32 gVendorId = 0;

static void updateGpuIds(QOpenGLFunctions *gl) {
  static bool firstCall = true;
  if (!firstCall)
    return;
  firstCall = false;
  D3DADAPTER_IDENTIFIER9 adapterinfo;
  LPDIRECT3D9 d3d_Object;

  d3d_Object = Direct3DCreate9(D3D_SDK_VERSION);
  d3d_Object->GetAdapterIdentifier(D3DADAPTER_DEFAULT, 0, &adapterinfo);
  gDeviceId = adapterinfo.DeviceId;
  gVendorId = adapterinfo.VendorId;
  d3d_Object->Release();
}
#endif

const void OmSysInfo::initializeOpenGlInfo() {
  openGLRenderer();
  openGLVendor();
  openGLVersion();
}

const QString &OmSysInfo::openGLRenderer() {
  static QString openGLRender;
  if (openGLRender.isEmpty())
    openGLRender = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
  return openGLRender;
}

const QString &OmSysInfo::openGLVendor() {
  static QString openGLVendor;
  if (openGLVendor.isEmpty())
    openGLVendor = reinterpret_cast<const char *>(glGetString(GL_VENDOR));
  return openGLVendor;
}

const QString &OmSysInfo::openGLVersion() {
  static QString openGLVersion;
  if (openGLVersion.isEmpty())
    openGLVersion = reinterpret_cast<const char *>(glGetString(GL_VERSION));
  return openGLVersion;
}

void OmSysInfo::openGlLineWidthRange(double &min, double &max) {
  GLfloat range[2];
  glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, range);
  min = range[0];
  max = range[1];
}

const QString &OmSysInfo::sysInfo() {
  static QString sysInfo;
  // cppcheck-suppress knownConditionTrueFalse
  if (!sysInfo.isEmpty())
    return sysInfo;

#ifdef _WIN32
  sysInfo.append(QSysInfo::prettyProductName());
  sysInfo.append(" ");

  SYSTEM_INFO winSysInfo;
  PGNSI pGetNativeSystemInfo =
    reinterpret_cast<PGNSI>(GetProcAddress(GetModuleHandle(TEXT("kernel32.dll")), "GetNativeSystemInfo"));
  if (NULL != pGetNativeSystemInfo)
    pGetNativeSystemInfo(&winSysInfo);
  else
    GetSystemInfo(&winSysInfo);

  if (winSysInfo.wProcessorArchitecture == PROCESSOR_ARCHITECTURE_INTEL)
    sysInfo.append("32-bit");
  else if (winSysInfo.wProcessorArchitecture == PROCESSOR_ARCHITECTURE_AMD64)
    sysInfo.append("64-bit");
  else if (winSysInfo.wProcessorArchitecture == PROCESSOR_ARCHITECTURE_IA64)
    sysInfo.append("Intel Itanium-based");
  else
    sysInfo.append("unknown architecture");
#else
  struct utsname buf;
  uname(&buf);
  sysInfo.append(buf.sysname);
  sysInfo.append(" ");
  sysInfo.append(buf.release);
  sysInfo.append(" ");
  sysInfo.append(buf.machine);
#endif

  return sysInfo;
}

OmSysInfo::OmPlatform OmSysInfo::platform() {
#ifdef __linux__
  return LINUX_PLATFORM;
#elif defined(__APPLE__)
  return MACOS_PLATFORM;
#elif defined(_WIN32)
  return WIN32_PLATFORM;
#else
#error unsupported platform
#endif
}

const QString &OmSysInfo::platformShortName() {
#ifdef __linux__
  static QString platformShortName;
  if (platformShortName.isEmpty()) {
    // cppcheck-suppress knownConditionTrueFalse
    if (OmSysInfo::isPointerSize64bits())
      platformShortName = "linux64";
    else
      platformShortName = "linux32";
  }
  return platformShortName;
#elif defined(__APPLE__)
  static const QString platformShortName = "mac";
  return platformShortName;
#elif defined(_WIN32)
  static const QString platformShortName = "windows";
  return platformShortName;
#else
#error unsupported platform
#endif
}

const QString &OmSysInfo::processor() {
  static QString processor;
  // cppcheck-suppress knownConditionTrueFalse
  if (!processor.isEmpty())
    return processor;
#ifdef _WIN32
  OmWindowsRegistry cpu("\\HKEY_LOCAL_MACHINE\\HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0");
  processor = cpu.stringValue("ProcessorNameString");
#elif defined(__APPLE__)
  size_t buflen = 100;
  char buf[buflen];
  sysctlbyname("machdep.cpu.brand_string", &buf, &buflen, NULL, 0);
  processor = buf;
#elif defined(__linux__)
  processor = linuxCpuModelName();
#endif
  return processor;
}

QString OmSysInfo::environmentVariable(const QString &name) {
#ifdef _WIN32  // on Windows, we cannot use the qgetenv function directly as it doesn't support UTF-8 characters
  wchar_t *wname = new wchar_t[name.length() + 1];
  name.toWCharArray(wname);
  wname[name.length()] = 0;
  int size = GetEnvironmentVariableW(wname, NULL, 0);
  if (size == 0)
    return QString();
  wchar_t *wvalue = new wchar_t[size];
  GetEnvironmentVariableW(wname, wvalue, size);
  delete[] wname;
  QString value = QString::fromWCharArray(wvalue);
  delete[] wvalue;
  return value;
#else
  return QString::fromUtf8(qgetenv(name.toUtf8()));
#endif
}

void OmSysInfo::setEnvironmentVariable(const QString &name, const QString &value) {
#ifdef _WIN32  // on Windows, we cannot use the qputenv function directly as it doesn't support UTF-8 characters
  wchar_t *wname = new wchar_t[name.length() + 1];
  name.toWCharArray(wname);
  wname[name.length()] = 0;
  wchar_t *wvalue = new wchar_t[value.length() + 1];
  value.toWCharArray(wvalue);
  wvalue[value.length()] = 0;
  SetEnvironmentVariableW(wname, wvalue);
#else
  qputenv(name.toUtf8(), value.toUtf8());
#endif
}

QString OmSysInfo::shortPath(const QString &path) {
#ifdef _WIN32
  wchar_t *input = new wchar_t[path.length() + 1];
  path.toWCharArray(input);
  input[path.length()] = 0;  // terminate string
  long length = GetShortPathNameW(input, NULL, 0);
  wchar_t *output = new wchar_t[length];
  GetShortPathNameW(input, output, length);
  QString ret = QString::fromWCharArray(output, length - 1);
  delete[] input;
  delete[] output;
  return ret;
#else
  return path;
#endif
}

// This function returns the number of CPU cores which may be different from
// the number of "logical cores" on some processors with hyperthreading
// (e.g., some Intel i7 have 4 CPU cores, but 8 logical cores thanks to
// hyperthreading). The value returned by QThread::idealThreadCount() is
// actually the number of logical cores. However, the optimal value for the
// number of threads in ODE MT is indeed the number of CPU cores. Thus, we
// need this function.
int OmSysInfo::coreCount() {
  static int coreCount = 0;
  if (coreCount > 0)
    return coreCount;

#ifdef _WIN32
  PSYSTEM_LOGICAL_PROCESSOR_INFORMATION buffer = NULL;
  DWORD returnLength = 0;
  GetLogicalProcessorInformation(buffer, &returnLength);
  buffer = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION)malloc(returnLength);
  GetLogicalProcessorInformation(buffer, &returnLength);
  int max = returnLength / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);
  coreCount = 0;
  for (int i = 0; i < max; i++)
    if (buffer[i].Relationship == RelationProcessorCore)
      coreCount++;
  free(buffer);

#elif defined(__linux__)
  coreCount = sysconf(_SC_NPROCESSORS_ONLN);

#elif defined(__APPLE__)
  kern_return_t kr;
  struct host_basic_info hostinfo;
  unsigned int count;

  count = HOST_BASIC_INFO_COUNT;
  kr = host_info(mach_host_self(), HOST_BASIC_INFO, (host_info_t)&hostinfo, &count);
  if (kr == KERN_SUCCESS)
    coreCount = hostinfo.avail_cpus;
#endif

  if (coreCount < 1)
    coreCount = 1;

  return coreCount;
}

#ifdef __linux__
const QString &OmSysInfo::linuxCpuModelName() {
  static QString cpuinfo;
  QFile cpuinfoFile("/proc/cpuinfo");
  if (cpuinfoFile.open(QIODevice::ReadOnly)) {
    const QStringList lines = QString(cpuinfoFile.readAll()).split('\n');
    foreach (const QString &line, lines) {
      if (line.startsWith("model name")) {
        // 12 corresponds to the strlen("model name: ")
        cpuinfo = line.mid(12).trimmed();  // remove leading and trailing whitespace
        break;
      }
    }
  }
  return cpuinfo;
}

bool OmSysInfo::isRootUser() {
  return geteuid() == 0;
}
#endif

bool OmSysInfo::isPointerSize32bits() {
  // cppcheck-suppress knownConditionTrueFalse
  return !OmSysInfo::isPointerSize64bits();
}

bool OmSysInfo::isPointerSize64bits() {
  return (sizeof(void *) == 8);
}

bool OmSysInfo::isVirtualMachine() {
  static char virtualMachine = -1;
  if (virtualMachine == 1)
    return true;
  if (virtualMachine == 0)
    return false;
  // list taken from https://www.techrepublic.com/blog/data-center/mac-address-scorecard-for-common-virtual-machine-platforms/
  // these are MAC addresses generated by default by virtual machines
  // however virtual machine also allow users to define a custom MAC address
  const QStringList virtualMachineMacIdentifiers = {
    "005056", "000C29", "000569",  // VMware ESX3, Server, Workstation, Player
    "0003FF",                      // Microsoft Hyper-V, Virtual Server, Virtual PC
    "001C42",                      // Parallells Desktop, Workstation, Server, Virtuozzo
    "000F4B",                      // Virtual Iron 4
    "00163E",                      // Red Hat Xen, Oracle VM, XenSource, Novell Xen
    "080027"                       // Sun xVM VirtualBox
  };
  const QString macAddress = OmMacAddress::instance()->address();
  foreach (const QString &id, virtualMachineMacIdentifiers) {
    if (macAddress.startsWith(id)) {
      virtualMachine = 1;
      return true;
    }
  }
// this is a more reliable way to determine if we are running on a virtual machine
#ifdef _WIN32
  unsigned int eax = 0, ebx = 0, ecx = 0, edx = 0;
  __get_cpuid(0x1, &eax, &ebx, &ecx, &edx);
  if (!(ecx & ((unsigned int)1 << 31))) {
    virtualMachine = 0;
    return false;
  }
  const auto queryVendorIdMagic = 0x40000000;
  __get_cpuid(queryVendorIdMagic, &eax, &ebx, &ecx, &edx);
  const int vendorIdLength = 13;
  using VendorIdStr = char[vendorIdLength];
  VendorIdStr hyperVendorId = {};
  // cppcheck-suppress nullPointer
  memcpy(hyperVendorId + 0, &ebx, 4);
  memcpy(hyperVendorId + 4, &ecx, 4);
  memcpy(hyperVendorId + 8, &edx, 4);
  hyperVendorId[12] = '\0';
  static const VendorIdStr vendors[]{
    "KVMKVMKVM\0\0\0",  // KVM
    "Microsoft Hv",     // Microsoft Hyper-V or Windows Virtual PC */
    "VMwareVMware",     // VMware
    "XenVMMXenVMM",     // Xen
    "prl hyperv  ",     // Parallels
    "VBoxVBoxVBox"      // VirtualBox
  };
  // cppcheck-suppress constVariableReference
  for (const auto &vendor : vendors) {
    if (!memcmp(vendor, hyperVendorId, vendorIdLength)) {
      virtualMachine = 1;
      return true;
    }
  }
  virtualMachine = 0;
  return false;
#else
#ifdef __linux__
  QFile cpuinfoFile("/proc/cpuinfo");
  if (!cpuinfoFile.open(QIODevice::ReadOnly)) {
    virtualMachine = 1;
    return true;  // unable to determine, assuming true
  }
  const QStringList lines = QString(cpuinfoFile.readAll()).split('\n');
  foreach (const QString &line, lines) {
    if (!line.startsWith("flags"))
      continue;
    const QStringList tokens = line.mid(line.indexOf(":") + 1).trimmed().split(" ");
    foreach (const QString &token, tokens)
      if (token == "hypervisor") {
        virtualMachine = 1;
        return true;
      }
    break;
  }
#endif
#ifdef __APPLE__
  if (system("ioreg -l | grep -e Manufacturer -e 'Vendor Name' | grep -E 'VMware|VirtualBox|Oracle|Parallels' > /dev/null") ==
      0) {
    virtualMachine = 1;
    return true;
  }
#endif
  virtualMachine = 0;
  return false;
#endif  // _WIN32
}

#ifdef _WIN32
quint32 OmSysInfo::gpuDeviceId(QOpenGLFunctions *gl) {
  updateGpuIds(gl);
  return gDeviceId;
}

quint32 OmSysInfo::gpuVendorId(QOpenGLFunctions *gl) {
  updateGpuIds(gl);
  return gVendorId;
}

#else

bool OmSysInfo::isLowEndGpu() {
  static char lowEndGpu = -1;  // not yet determined
  if (lowEndGpu == -1) {       // heuristic based on historical Webots telemetry of common GPU models
    lowEndGpu = 0;
    const QString &renderer = openGLRenderer();
    if (renderer.contains("Intel") && renderer.contains(" HD Graphics ")) {
      // we support only recent Intel GPUs from about 2015
      if (renderer.contains("Ivybridge") || renderer.contains("Sandybridge") || renderer.contains("Haswell") ||
          renderer.contains("Ironlake"))
        lowEndGpu = 1;
      else {
        const QRegularExpression re(" HD Graphics P{0,1}([\\d]{3,4})");
        const QRegularExpressionMatch match = re.match(renderer);
        const int number = match.hasMatch() ? match.captured(1).toInt() : 0;

        if ((number >= 2000 && number <= 6000) || (number >= 100 && number < 500))
          lowEndGpu = 1;
      }
    } else if (renderer.contains("Radeon HD") || renderer.contains("Radeon(TM) HD"))
      lowEndGpu = 1;  // We don't support old AMD Radeon HD cards
  }
  return (bool)lowEndGpu;
}

#endif
