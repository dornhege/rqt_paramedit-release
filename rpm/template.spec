Name:           ros-indigo-rqt-paramedit
Version:        1.0.0
Release:        2%{?dist}
Summary:        ROS rqt_paramedit package

Group:          Development/Libraries
License:        BSD
URL:            http://ros.org/wiki/rqt_paramedit
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-indigo-qt-paramedit
Requires:       ros-indigo-roscpp
Requires:       ros-indigo-rqt-gui
Requires:       ros-indigo-rqt-gui-cpp
BuildRequires:  ros-indigo-catkin
BuildRequires:  ros-indigo-qt-paramedit
BuildRequires:  ros-indigo-roscpp
BuildRequires:  ros-indigo-rqt-gui
BuildRequires:  ros-indigo-rqt-gui-cpp

%description
rqt_paramedit - a rqt plugin for editing parameters using qt_paramedit.

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_LIBDIR="lib" \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/indigo" \
        -DCMAKE_PREFIX_PATH="/opt/ros/indigo" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/indigo

%changelog
* Wed Apr 13 2016 Christian Dornhege <c.dornhege@gmail.com> - 1.0.0-2
- Autogenerated by Bloom

* Wed Apr 13 2016 Christian Dornhege <c.dornhege@gmail.com> - 1.0.0-1
- Autogenerated by Bloom

* Wed Apr 13 2016 Christian Dornhege <c.dornhege@gmail.com> - 1.0.0-0
- Autogenerated by Bloom

