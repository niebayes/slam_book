/***************************************************************************
 *  Description: import/export macros for creating DLLS with Microsoft
 *	compiler. Any exported function needs to be declared with the
 *  appropriate G2O_XXXX_API macro. Also, there must be separate macros
 *  for each DLL (arrrrrgh!!!)
 *
 *  17 Jan 2012
 *  Email: pupilli@cs.bris.ac.uk
 ****************************************************************************/
#ifndef G2O_CORE_API_H
#define G2O_CORE_API_H

#include "g2o/config.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#ifdef core_EXPORTS

// * C++ attributes: an attribute acts as an annotation or a note to the
// * compiler which provides additional information about the code for
// * optimization purposes and enforcing certain conditions on it.
// ! warning: attributes are compiler-specific.

// * The dllexport and dllimport storage-class attributes are Microsoft-specific
// * extensions to the C and C++ languages. You can use them to export and
// * import functions, data, and objects to or from a DLL.
// * This trick is most apparent when trying to export decorated C++ function
// * names.
// * cf.
// * https://docs.microsoft.com/en-us/cpp/cpp/dllexport-dllimport?view=vs-2019
#define G2O_CORE_API __declspec(dllexport)
#else
#define G2O_CORE_API __declspec(dllimport)
#endif
#else
#define G2O_CORE_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define G2O_CORE_API
#endif

#endif  // G2O_CORE_API_H
