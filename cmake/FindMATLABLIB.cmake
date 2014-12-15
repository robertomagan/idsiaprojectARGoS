# Try to find GALIB
# See http://lancet.mit.edu/ga/
#
# Once run this will define: 
# 
# GALIB_FOUND        = system has GALIB lib
#
# GALIB_LIBRARIES    = full path to the libraries
# GALIB_INCLUDE_DIR  = where to find headers 


#
# Find the library
#
FIND_LIBRARY(MATLABLIB_LIBRARY
  NAMES
  eng mx
  PATHS
  /usr/local/MATLAB/R2010b/bin/glnxa64
  DOC "Google PerfTools library location"
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MATLABLIB DEFAULT_MSG
	MATLABLIB_LIBRARY)

IF(MATLABLIB_LIBRARY)	
	SET(MATLABLIB_LIBRARIES	  ${MATLABLIB_LIBRARY})
ENDIF(MATLABLIB_LIBRARY)

MARK_AS_ADVANCED(MATLABLIB_LIBRARY)
