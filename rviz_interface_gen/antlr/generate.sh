#!/bin/bash

PKG_PATH=`rospack find rviz_interface_gen`
java -classpath $PKG_PATH/antlr/antlr-3.1.2.jar org.antlr.Tool -o $PKG_PATH/src/rviz_interface_gen $PKG_PATH/antlr/InterfaceLexer.g $PKG_PATH/antlr/InterfaceParser.g