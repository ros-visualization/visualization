#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib; roslib.load_manifest('rviz_interface_gen')
import roslib.msgs
import roslib.packages

import rviz_interface_gen
from rviz_interface_gen.InterfaceParser import InterfaceParser
from rviz_interface_gen.InterfaceLexer import InterfaceLexer
from rviz_interface_gen.antlr3 import ANTLRFileStream
from rviz_interface_gen.antlr3 import CommonTokenStream

from cStringIO import StringIO

import os
import os.path
import sys

def msg_prefix_from_interface_method(i, m):
    return '%s_%s'%(i.name, m.name)

def get_generated_messages(i):
    msgs = []
    for m in i.methods:
        request_message = '%sRequest'%(msg_prefix_from_interface_method(i, m))
        response_message = '%sResponse'%(msg_prefix_from_interface_method(i, m))
        msgs = msgs + [request_message, response_message]
        
    return msgs

def write_message(i, members, pkg, pkg_path, name):
    output_dir = '%s/msg'%(pkg_path)
    if (not os.path.exists(output_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(output_dir)
        except OSError, e:
            pass
         
    f = open('%s/%s.msg'%(output_dir, name), 'w')
    
    s = StringIO()
    for m in members:
        print >> s, '%s %s'%(m[0], m[1])
        
    print >> f, s.getvalue()
    f.close()

def write_method_messages(i, m, pkg, pkg_path):
    request_message = '%sRequest'%(msg_prefix_from_interface_method(i, m))
    response_message = '%sResponse'%(msg_prefix_from_interface_method(i, m))
    write_message(i, m.fields, pkg, pkg_path, request_message)
    write_message(i, m.return_fields, pkg, pkg_path, response_message)

def generate_interface(i, pkg, pkg_path):
    for m in i.methods:
        write_method_messages(i, m, pkg, pkg_path)

def generate(interface_file, pkg, pkg_path):
    parser = load_from_file(interface_file)
    [generate_interface(i, pkg, pkg_path) for i in parser.interfaces]

def load_from_file(interface_file):
    char_stream = ANTLRFileStream(sys.argv[1])
    lexer = InterfaceLexer(char_stream)
    tokens = CommonTokenStream(lexer)
    parser = InterfaceParser(tokens);
    parser.main()
    
    return parser