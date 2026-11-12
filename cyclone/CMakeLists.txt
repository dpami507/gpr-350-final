cmake_minimum_required(VERSION 3.25)
project(cyclone)
set(cyclone_sources
	"core.cpp"

	"particle.cpp"
	"particleForceRegistry.cpp"

	"particleGravity.cpp"
	"particleDrag.cpp"
	"particleSpring.cpp" 
	"particleBouyancy.cpp"
	"rigidbody.cpp" 
	"collider.cpp" 
	"contacts.cpp"
	"contactResolver.cpp"
	"octree.cpp" 
	"spacialHashing.cpp" 
	"verletParticle.cpp" 
	"rope.cpp"
	"gravityBody.cpp")
set(cyclone_headers
	"precision.h"
	"core.h"
	"cyclone.h"

	"particle.h"
	"particleForceGen.h"
	"particleForceRegistry.h"

	"particleGravity.h"
	"particleDrag.h"
	"particleSpring.h" 
	"particleBouyancy.h"
	"rigidbody.h" 
	"collider.h" 
	"contacts.h"
	"contactResolver.h"
	"octree.h" 
	"spacialHashing.h" 
	"verletParticle.h" 
	"rope.h"
	"gravityBody.h")

set(cyclone_INC_DIR
 ${CMAKE_CURRENT_SOURCE_DIR}
 CACHE PATH "CYCLONE INCLUDE SOURCE PATH"
)
add_library(cyclone STATIC ${cyclone_sources} ${cyclone_headers} "collider.h")
target_include_directories(cyclone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(cyclone PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
install (TARGETS cyclone DESTINATION lib)
install (FILES ${cyclone_headers} DESTINATION include/cyclone)
file(COPY ${cyclone_headers} DESTINATION "include")