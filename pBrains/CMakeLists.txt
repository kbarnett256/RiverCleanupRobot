#--------------------------------------------------------
# The CMakeLists.txt for:                         pBrains
# Author(s):                              Kenneth Barnett
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
     wsock32 )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m
      pthread )
endif (${WIN32})


SET(SRC
   main.cpp  
   pBrains.cpp
)  

ADD_EXECUTABLE(pBrains ${SRC})

TARGET_LINK_LIBRARIES(pBrains
   ${MOOS_LIBRARIES}
   mbutil
   ${SYSTEM_LIBS} )
