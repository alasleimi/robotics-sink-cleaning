cmake_minimum_required(VERSION 3.15)
project(pglfd VERSION "0.0.0")

option(BUILD_PYTHON "create python bindings using pybind11" ON)
option(WITH_TBB "use tbb for parallelism" ON)
option(WITH_OPENMP "use OpenMP for parallelism" ON)

# Set the C++ standard to C++23
set(CMAKE_CXX_STANDARD 23)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)

add_library(pglfd INTERFACE)
add_library(pglfd::pglfd ALIAS pglfd)
target_link_libraries(pglfd INTERFACE Eigen3::Eigen)
target_compile_definitions(pglfd INTERFACE EIGEN_MAX_ALIGN_BYTES=16)

if(${WITH_OPENMP})
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        set(OpenMP_DEPENDENCY "find_dependency(OpenMP ${OpenMP_VERSION})")
        target_link_libraries(pglfd INTERFACE OpenMP::OpenMP_CXX)
    else()
        set(OpenMP_DEPENDENCY "")
    endif()
endif()

if(${WITH_TBB})
    find_package(TBB)
    if(TBB_FOUND)
        set(TBB_DEPENDENCY "find_dependency(TBB ${TBB_VERSION})")
        target_link_libraries(pglfd INTERFACE TBB::tbb)
        target_compile_definitions(pglfd INTERFACE HAVE_TBB)
    else()
        set(TBB_DEPENDENCY "")
    endif()
endif()

set(PBLFD_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/pglfd/cmake)

# Associate target with include directory
target_include_directories(pglfd INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
        )

install(TARGETS pglfd EXPORT pglfdTargets)
install(EXPORT pglfdTargets
        NAMESPACE pglfd::
        DESTINATION ${PBLFD_CMAKE_EXPORT_DIR}
        )

export(TARGETS pglfd NAMESPACE pglfd:: FILE pglfdTargets.cmake)
export(PACKAGE pglfd)

configure_package_config_file(
        pglfdConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/pglfdConfig.cmake
        INSTALL_DESTINATION ${PBLFD_CMAKE_EXPORT_DIR}
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Write version to file
write_basic_package_version_file(
        pglfdConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
)

# Recover architecture dependence
set(CMAKE_SIZEOF_VOID_P ${TEMP_SIZEOF_VOID_P})

# Install cmake targets
install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/pglfdConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/pglfdConfigVersion.cmake
        DESTINATION ${PBLFD_CMAKE_EXPORT_DIR}
)

# Install header files
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/)

# Python bindings, if they exists
find_package(pybind11)
if(pybind11_FOUND AND BUILD_PYTHON)
    set(GENERATED_SOURCES
        "${CMAKE_CURRENT_BINARY_DIR}/conditioned_gaussian.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/gaussian_regressor.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/gaussian.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/gmm.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/gmr.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/gp.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/kernel.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/kmeans.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/kmp.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/lqt.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/nw.cpp"
        "${CMAKE_CURRENT_BINARY_DIR}/tp.cpp"
    )

    add_custom_command(
        COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/create_pybind.py ${CMAKE_CURRENT_BINARY_DIR}
        OUTPUT ${GENERATED_SOURCES}
        COMMENT "Generating code for python bindings"
    )

    # python wrappers
    find_package(Python 3 REQUIRED)
    file(GLOB PY_FILES "src/pybind/*.py")
    install(FILES ${PY_FILES} DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # kmeans
    pybind11_add_module(kmeans ${CMAKE_CURRENT_BINARY_DIR}/kmeans.cpp)
    target_link_libraries(kmeans PRIVATE pglfd::pglfd)
    install(TARGETS kmeans DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # gaussian
    pybind11_add_module(gaussian ${CMAKE_CURRENT_BINARY_DIR}/gaussian.cpp)
    target_link_libraries(gaussian PRIVATE pglfd::pglfd)
    install(TARGETS gaussian DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # gaussian_regressor
    pybind11_add_module(gaussian_regressor ${CMAKE_CURRENT_BINARY_DIR}/gaussian_regressor.cpp)
    target_link_libraries(gaussian_regressor PRIVATE pglfd::pglfd)
    install(TARGETS gaussian_regressor DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # conditioned_gaussian
    pybind11_add_module(conditioned_gaussian ${CMAKE_CURRENT_BINARY_DIR}/conditioned_gaussian.cpp)
    target_link_libraries(conditioned_gaussian PRIVATE pglfd::pglfd)
    install(TARGETS conditioned_gaussian DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # gmm
    pybind11_add_module(gmm ${CMAKE_CURRENT_BINARY_DIR}/gmm.cpp)
    target_link_libraries(gmm PRIVATE pglfd::pglfd)
    install(TARGETS gmm DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # gmr
    pybind11_add_module(gmr ${CMAKE_CURRENT_BINARY_DIR}/gmr.cpp)
    target_link_libraries(gmr PRIVATE pglfd::pglfd)
    install(TARGETS gmr DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # kernel
    pybind11_add_module(kernel ${CMAKE_CURRENT_BINARY_DIR}/kernel.cpp)
    target_link_libraries(kernel PRIVATE pglfd::pglfd)
    install(TARGETS kernel DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # gp
    pybind11_add_module(gp ${CMAKE_CURRENT_BINARY_DIR}/gp.cpp)
    target_link_libraries(gp PRIVATE pglfd::pglfd)
    install(TARGETS gp DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # kmp
    pybind11_add_module(kmp ${CMAKE_CURRENT_BINARY_DIR}/kmp.cpp)
    target_link_libraries(kmp PRIVATE pglfd::pglfd)
    install(TARGETS kmp DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # nadaraya_watson
    pybind11_add_module(nw ${CMAKE_CURRENT_BINARY_DIR}/nw.cpp)
    target_link_libraries(nw PRIVATE pglfd::pglfd)
    install(TARGETS nw DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # lqt
    pybind11_add_module(lqt ${CMAKE_CURRENT_BINARY_DIR}/lqt.cpp)
    target_link_libraries(lqt PRIVATE pglfd::pglfd)
    install(TARGETS lqt DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)

    # tp
    pybind11_add_module(tp ${CMAKE_CURRENT_BINARY_DIR}/tp.cpp)
    target_link_libraries(tp PRIVATE pglfd::pglfd)
    install(TARGETS tp DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/pglfd_cpp)
endif()

# Testing
enable_testing()

# Geom
add_executable(geom_so2_test test/geom/so2.cpp)
target_link_libraries(geom_so2_test pglfd)
add_test(NAME geom_so2_test COMMAND geom_so2_test)

add_executable(geom_s2_test test/geom/s2.cpp)
target_link_libraries(geom_s2_test pglfd)
add_test(NAME geom_s2_test COMMAND geom_s2_test)

add_executable(geom_so3_test test/geom/so3.cpp)
target_link_libraries(geom_so3_test pglfd)
add_test(NAME geom_so3_test COMMAND geom_so3_test)

add_executable(geom_se3_test test/geom/se3.cpp)
target_link_libraries(geom_se3_test pglfd)
add_test(NAME geom_se3_test COMMAND geom_se3_test)

# Manifolds
add_executable(manifold_euclidean_test test/manif/euclidean.cpp)
target_link_libraries(manifold_euclidean_test pglfd)
add_test(NAME manifold_euclidean_test COMMAND manifold_euclidean_test)

add_executable(manifold_so2_test test/manif/so2.cpp)
target_link_libraries(manifold_so2_test pglfd)
add_test(NAME manifold_so2_test COMMAND manifold_so2_test)

add_executable(manifold_s2_test test/manif/s2.cpp)
target_link_libraries(manifold_s2_test pglfd)
add_test(NAME manifold_s2_test COMMAND manifold_s2_test)

add_executable(manifold_so3_test test/manif/so3.cpp)
target_link_libraries(manifold_so3_test pglfd)
add_test(NAME manifold_so3_test COMMAND manifold_so3_test)

add_executable(manifold_product_test test/manif/product.cpp)
target_link_libraries(manifold_product_test pglfd)
add_test(NAME manifold_product_test COMMAND manifold_product_test)

add_executable(manifold_conversions_test test/manif/conversions.cpp)
target_link_libraries(manifold_conversions_test pglfd)
add_test(NAME manifold_conversions_test COMMAND manifold_conversions_test)

add_executable(manifold_jacobian_test test/manif/jacobian.cpp)
target_link_libraries(manifold_jacobian_test pglfd)
add_test(NAME manifold_jacobian_test COMMAND manifold_jacobian_test)

# KMeans
add_executable(kmeans_test test/model/kmeans.cpp)
target_link_libraries(kmeans_test pglfd)
add_test(NAME kmeans_test COMMAND kmeans_test)

# Gaussian
add_executable(gaussian_test test/model/gaussian.cpp)
target_link_libraries(gaussian_test pglfd)
add_test(NAME gaussian_test COMMAND gaussian_test)

# GMM
add_executable(gmm_test test/model/gmm.cpp)
target_link_libraries(gmm_test pglfd)
add_test(NAME gmm_test COMMAND gmm_test)

# GP
add_executable(gp_test test/model/gp.cpp)
target_link_libraries(gp_test pglfd)
add_test(NAME gp_test COMMAND gp_test)

# KMP
add_executable(kmp_test test/model/kmp.cpp)
target_link_libraries(kmp_test pglfd)
add_test(NAME kmp_test COMMAND kmp_test)

# LQT
add_executable(lqt_test test/model/lqt.cpp)
target_link_libraries(lqt_test pglfd)
add_test(NAME lqt_test COMMAND lqt_test)

# NW
add_executable(nw_test test/model/nw.cpp)
target_link_libraries(nw_test pglfd)
add_test(NAME nw_test COMMAND nw_test)

# TP
add_executable(tp_test test/model/tp.cpp)
target_link_libraries(tp_test pglfd)
add_test(NAME tp_test COMMAND tp_test)

# get all targets
function(get_all_targets var)
    set(targets)
    get_all_targets_recursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
    set(${var} ${targets} PARENT_SCOPE)
endfunction()

macro(get_all_targets_recursive targets dir)
    get_property(subdirectories DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
    foreach(subdir ${subdirectories})
        get_all_targets_recursive(${targets} ${subdir})
    endforeach()

    get_property(current_targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
    list(APPEND ${targets} ${current_targets})
endmacro()

get_all_targets(all_targets)

# set cppcheck for all targets
foreach(targ ${all_targets})
	set_property(TARGET ${targ} PROPERTY CXX_CPPCHECK "cppcheck" "--enable=all" "--disable=unusedFunction" "--suppress=missingIncludeSystem" "--suppress=checkersReport" "--suppress=unmatchedSuppression" "--inline-suppr" "--std=c++20" "--xml" "--inconclusive" "--output-file=${targ}-cppcheck.xml")
endforeach()

get_all_targets(all_targets)
message("All targets: ${all_targets}")
