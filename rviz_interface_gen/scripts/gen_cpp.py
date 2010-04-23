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
import roslib.packages
import rviz_interface_gen.msgs as gen
import roslib.msgs as msgs
import roscpp.msg_gen as genmsg_cpp

from cStringIO import StringIO

import sys
import os

def write_header_begin(hs, i, pkg):
    print >> hs, '#ifndef INTERFACE_%s_%s_H'%(pkg.upper(), i.name.upper())
    print >> hs, '#define INTERFACE_%s_%s_H\n'%(pkg.upper(), i.name.upper())
    
def write_header_end(hs, i, pkg):
    print >> hs, '#endif // INTERFACE_%s_%s_H'%(pkg.upper(), i.name.upper())
    
def write_header_includes(hs, i, pkg):
    print >> hs, '#include <ros/time.h>'
    for m in i.methods:
        for f in m.fields:
            (base_type, is_array, array_len) = msgs.parse_type(f[0])
            if (is_array):
                raise Error("Arrays are not supported as parameter or return-values")
            
            if (not msgs.is_builtin(base_type)):
                resolved = msgs.resolve_type(base_type, pkg)
                print >> hs, '#include <%s.h>'%(resolved)
                
    print >> hs

def msg_to_cpp(type):
    if (msgs.is_builtin(type)):
        return genmsg_cpp.msg_type_to_cpp(type)
    else:
        ns = type.split('/')[0]
        c = type.split('/')[1]
        return '%s::%s'%(ns, c)  

def write_header_method_declaration(hs, i, m, pure_virtual):
    print >> hs, '  virtual void %s('%(m.name),
    
    first = True
    for f in m.fields:
        (type, _, _) = msgs.parse_type(f[0])
        name = f[1]
        
        if (first):
           first = False
        else:
            print >> hs, ', ',
        
        cpp_type = msg_to_cpp(type)
        if (msgs.is_valid_constant_type(type)):
            print >> hs, '%s %s'%(cpp_type, name),
        else:
            print >> hs, 'const %s& %s'%(cpp_type, name),
            
    for f in m.return_fields:
        (type, _, _) = msgs.parse_type(f[0])
        name = f[1]
        
        if (first):
           first = False
        else:
            print >> hs, ', ',
        
        cpp_type = msg_to_cpp(type)
        print >> hs, '%s& out_%s'%(cpp_type, name),
             
    print >> hs, ')',
    if (pure_virtual):
        print >> hs, ' = 0',
    print >> hs, ';'

def write_header_interface_declaration(hs, i, pkg):
    print >> hs, 'struct %s\n{'%(i.name)
    
    # Write virtual destructor
    print >> hs, '  virtual ~%s() {}'%(i.name)
    
    for m in i.methods:
        write_header_method_declaration(hs, i, m, True)
    
    print >> hs, '};\n'
    
def write_header_proxy_declaration(hs, i, pkg):
    print >> hs, 'struct %sProxy\n{'%(i.name)
    
    print >> hs, '  %sProxy();'%(i.name)
    print >> hs, '  void connect(const std::string& name, const ros::NodeHandle& nh);'
    
    for m in i.methods:
        write_header_method_declaration(hs, i, m, False)
        
    print >> hs, 'private:'
    print >> hs, '  struct Impl;'
    print >> hs, '  typedef boost::shared_ptr<Impl> ImplPtr;'
    print >> hs, '  ImplPtr impl_;'
    
    print >> hs, '};\n'
    
def write_header_server_declaration(hs, i, pkg):
    print >> hs, 'struct %sServer\n{'%(i.name)
    
    print >> hs, '  %sServer();'%(i.name)
    
    for m in i.methods:
        write_header_method_declaration(hs, i, m, False)
        
    print >> hs, 'private:'
    print >> hs, '  struct Impl;'
    print >> hs, '  typedef boost::shared_ptr<Impl> ImplPtr;'
    print >> hs, '  ImplPtr impl_;'
    
    print >> hs, '};\n'

def generate(output_dir, i, pkg, pkg_path):
    hs = StringIO()
    cpps = StringIO()
    
    write_header_begin(hs, i, pkg)
    write_header_includes(hs, i, pkg)
    
    print >> hs, 'namespace %s\n{\n'%(pkg)
    
    write_header_interface_declaration(hs, i, pkg)
    write_header_proxy_declaration(hs, i, pkg)
    write_header_server_declaration(hs, i, pkg)
    
    print >> hs, '}\n'
    
    write_header_end(hs, i, pkg)
    
    try:
        os.makedirs('%s/include'%(output_dir))
    except OSError, e:
        pass
    try:
        os.makedirs('%s/src'%(output_dir))
    except OSError, e:
        pass
         
    f = open('%s/include/%s.h'%(output_dir, i.name), 'w')
    print >> f, hs.getvalue()
    f.close()
    
    f = open('%s/src/%s.cpp'%(output_dir, i.name), 'w')
    print >> f, cpps.getvalue()
    f.close()

if __name__ == "__main__":
    input = sys.argv[1]
    output_dir = sys.argv[2]
        
    (package_dir, package) = roslib.packages.get_dir_pkg(input)
    p = gen.load_from_file(input)
    for i in p.interfaces:
        generate(output_dir, i, package, package_dir)
        