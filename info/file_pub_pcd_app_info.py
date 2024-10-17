#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'FILE_PUB_PCD' # Use in display menus
FILE_TYPE = 'APP'
APP_DICT = dict(
    description = 'Application for publishing pointclouds from pcd files',
    pkg_name = 'nepi_app_file_pub_pcd',
    group_name = 'FILE_PUB',
    config_file = 'app_file_pub_pcd.yaml',
    app_file = 'file_pub_pcd_app_node.py',
    node_name = 'app_file_pub_pcd'
)
RUI_DICT = dict(
    rui_menu_name = "Pointcloud Publisher", # RUI menu name or "None" if no rui support
    rui_files = ['NepiAppFilePubPcd.js'],
    rui_main_file = "NepiAppFilePubPcd.js",
    rui_main_class = "FilePubPcdApp"
)




