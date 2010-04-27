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
    print >> hs, '#include <rviz_interface_gen/interface.h>'
    print >> hs, '#include <ros/time.h>'
    print >> hs, '#include <string>'
    
    headers = []
    for m in i.methods:
        for f in m.fields + m.return_fields:
            (base_type, is_array, array_len) = msgs.parse_type(f[0])
            if (is_array):
                raise Error("Arrays are not supported as parameter or return-values")
            
            if (not msgs.is_builtin(base_type)):
                resolved = msgs.resolve_type(base_type, pkg)
                headers.append(resolved)
                
    headers = set(headers)
    for h in headers:
        print >> hs, '#include <%s.h>'%(h)
    print >> hs

def msg_to_cpp(type):
    if (msgs.is_builtin(type)):
        return genmsg_cpp.msg_type_to_cpp(type)
    else:
        ns = type.split('/')[0]
        c = type.split('/')[1]
        return '%s::%s'%(ns, c)
    
def write_header_forward_declarations(hs):
    print >> hs, 'namespace ros { class NodeHandle; }'
    print >> hs, 'namespace ros { template<typename M> class MessageEvent; }'
    print >> hs
    
def write_method_args(s, i, m):
    first = True
    for f in m.fields:
        (type, _, _) = msgs.parse_type(f[0])
        name = f[1]
        
        if (first):
           first = False
        else:
            print >> s, ', ',
        
        cpp_type = msg_to_cpp(type)
        if (msgs.is_valid_constant_type(type)):
            print >> s, '%s %s'%(cpp_type, name),
        else:
            print >> s, 'const %s& %s'%(cpp_type, name),
            
    for f in m.return_fields:
        (type, _, _) = msgs.parse_type(f[0])
        name = f[1]
        
        if (first):
           first = False
        else:
            print >> s, ', ',
        
        cpp_type = msg_to_cpp(type)
        print >> s, '%s& out_%s'%(cpp_type, name),

def write_header_method_declaration(hs, i, m, pure_virtual):
    print >> hs, '  virtual void %s('%(m.name),
    
    write_method_args(hs, i, m)
             
    print >> hs, ')',
    if (pure_virtual):
        print >> hs, ' = 0',
    print >> hs, ';'

def write_header_interface_declaration(hs, i, pkg):
    print >> hs, 'struct %s : public rviz_interface_gen::Interface\n{'%(i.name)
    
    # Write virtual destructor
    print >> hs, '  virtual ~%s() {}'%(i.name)
    
    for m in i.methods:
        write_header_method_declaration(hs, i, m, True)
    
    print >> hs, '};\n'
    
def write_header_proxy_declaration(hs, i, pkg):
    print >> hs, 'struct %sProxy : public %s\n{'%(i.name, i.name)
    
    print >> hs, '  %sProxy(const std::string& name, const ros::NodeHandle& nh);'%(i.name)
    
    for m in i.methods:
        write_header_method_declaration(hs, i, m, False)
        
    print >> hs, 'private:'
    print >> hs, '  struct Impl;'
    print >> hs, '  typedef boost::shared_ptr<Impl> ImplPtr;'
    print >> hs, '  ImplPtr impl_;'
    
    print >> hs, '};\n'
    
def write_header_server_declaration(hs, i, pkg):
    print >> hs, 'struct %sServer : public %s\n{'%(i.name, i.name)
    
    print >> hs, '  %sServer(const std::string& name, const ros::NodeHandle& nh);'%(i.name)
        
    print >> hs, 'private:'
    print >> hs, '  struct Impl;'
    print >> hs, '  typedef boost::shared_ptr<Impl> ImplPtr;'
    print >> hs, '  ImplPtr impl_;'
    
    print >> hs, '};\n'
    
    
def write_cpp_includes(cpps, i, pkg):
    print >> cpps, '#include <%s/%s.h>'%(pkg, i.name)
    print >> cpps, '#include <rviz_rpc/client.h>'
    print >> cpps, '#include <rviz_rpc/server.h>'
    
    for m in i.methods:
        print >> cpps, '#include <%s/%s_%sRequest.h>'%(pkg, i.name, m.name)
        print >> cpps, '#include <%s/%s_%sResponse.h>'%(pkg, i.name, m.name)
        
    print >> cpps
    print >> cpps, 'using namespace %s;\n'%(pkg)

def write_cpp_proxy_method(cpps, i, m, pkg):
    print >> cpps, 'void %sProxy::%s('%(i.name, m.name),
    write_method_args(cpps, i, m)
    print >> cpps, ')',
    cpp_prefix = cpp_message_prefix(pkg, i, m)
    print >> cpps, """
{
  %sRequestPtr req(new %sRequest);
"""%(cpp_prefix, cpp_prefix)

    for field in m.fields:
        print >> cpps, '  req->%s = %s;'%(field[1], field[1])
        
    if ('async' in m.attributes):
        if (len(m.return_fields) > 0):
            raise Error('Calls with return values cannot be async (method %s_%s)'%(i.name, m.name))
        
        print >> cpps, '  impl_->%s_method_.callAsync(req);'%(m.name)
    else:
        print >> cpps, '  %sResponseConstPtr res = impl_->%s_method_.call(req);'%(cpp_prefix, m.name)
        for field in m.return_fields:
            print >> cpps, '  out_%s = res->%s;'%(field[1], field[1])
            
    print >> cpps, '}\n'

def cpp_message_prefix(pkg, i, m):
    return '%s::%s_%s'%(pkg, i.name, m.name)

def write_cpp_proxy_definition(cpps, i, pkg):    
    print >> cpps, 'struct %sProxy::Impl\n{'%(i.name)
    print >> cpps, """ 
  Impl(const std::string& name, const ros::NodeHandle& nh)
  : client_(name, nh)
  {
  }
"""

    print >> cpps, '  rviz_rpc::Client client_;'
    for m in i.methods:
        print >> cpps, '  rviz_rpc::Method<%sRequest, %sResponse> %s_method_;'%(cpp_message_prefix(pkg, i, m), cpp_message_prefix(pkg, i, m), m.name)
    print >> cpps, '};\n'
    
    print >> cpps, '%sProxy::%sProxy(const std::string& name, const ros::NodeHandle& nh)'%(i.name, i.name)
    print >> cpps, ': impl_(new Impl(name, nh))\n{'
    for m in i.methods:
        print >> cpps, '  impl_->%s_method_ = impl_->client_.addMethod<%sRequest, %sResponse>("%s");'%(m.name, cpp_message_prefix(pkg, i, m), cpp_message_prefix(pkg, i, m), m.name)
        
    print >> cpps, '  impl_->client_.connect();'
    print >> cpps, '}\n'
    
    for m in i.methods:
        write_cpp_proxy_method(cpps, i, m, pkg)
        
def write_cpp_server_definition(cpps, i, pkg):
    print >> cpps, 'struct %sServer::Impl\n{'%(i.name)
    print >> cpps, """
  Impl(const std::string& name, const ros::NodeHandle& nh, %sServer* parent)
  : server_(name, nh)
  , parent_(parent)
  {
  }
"""%(i.name)

    for m in i.methods:
        cpp_prefix = cpp_message_prefix(pkg, i, m)
        print >> cpps, '  %sResponseConstPtr %s_callback(const ros::MessageEvent<%sRequest>& evt)\n  {'%(cpp_prefix, m.name, cpp_prefix)
        print >> cpps, '    %sRequestPtr req = evt.getMessage();'%(cpp_prefix)
        print >> cpps, '    %sResponsePtr res(new %sResponse);'%(cpp_prefix, cpp_prefix)
        print >> cpps, '    parent_->%s('%(m.name),
        first = True
        for f in m.fields:
            if (first):
                first = False
            else:
                print >> cpps, ',',
            print >> cpps, 'req->%s'%(f[1]),
            
        for f in m.return_fields:
            if (first):
                first = False
            else:
                print >> cpps, ',',
            
            print >> cpps, 'res->%s'%(f[1]),
        print >> cpps, ');\n'
        print >> cpps, '    return res;'
        print >> cpps, '  }'

    print >> cpps
    print >> cpps, '  rviz_rpc::Server server_;'
    print >> cpps, '  %sServer* parent_;'%(i.name)
    print >> cpps, '};\n'
    
    print >> cpps, '%sServer::%sServer(const std::string& name, const ros::NodeHandle& nh)'%(i.name, i.name)
    print >> cpps, ': impl_(new Impl(name, nh, this))'
    print >> cpps, '{'
    for m in i.methods:
        cpp_prefix = cpp_message_prefix(pkg, i, m)
        print >> cpps, '  impl_->server_.addMethod<%sRequest, %sResponse>("%s", boost::bind(&Impl::%s_callback, impl_.get(), _1));'%(cpp_prefix, cpp_prefix, m.name, m.name)
    print >> cpps, '  impl_->server_.ready();'
    print >> cpps, '}'
    

def generate(output_dir, i, pkg, pkg_path):
    hs = StringIO()
    cpps = StringIO()
    
    write_header_begin(hs, i, pkg)
    write_header_includes(hs, i, pkg)
    
    write_header_forward_declarations(hs)
    
    print >> hs, 'namespace %s\n{\n'%(pkg)
    
    write_header_interface_declaration(hs, i, pkg)
    write_header_proxy_declaration(hs, i, pkg)
    write_header_server_declaration(hs, i, pkg)
    
    print >> hs, '}\n'
    
    write_header_end(hs, i, pkg)
    
    write_cpp_includes(cpps, i, pkg)
    write_cpp_proxy_definition(cpps, i, pkg)
    write_cpp_server_definition(cpps, i, pkg)
    
    header_dir = '%s/include/%s'%(output_dir, pkg)
    cpp_dir = '%s/src'%(output_dir)
    try:
        os.makedirs(header_dir)
    except OSError, e:
        pass
    try:
        os.makedirs(cpp_dir)
    except OSError, e:
        pass
         
    f = open('%s/%s.h'%(header_dir, i.name), 'w')
    print >> f, hs.getvalue()
    f.close()
    
    f = open('%s/%s.cpp'%(cpp_dir, i.name), 'w')
    print >> f, cpps.getvalue()
    f.close()

if __name__ == "__main__":
    input = sys.argv[1]
    output_dir = sys.argv[2]
        
    (package_dir, package) = roslib.packages.get_dir_pkg(input)
    p = gen.load_from_file(input)
    for i in p.interfaces:
        generate(output_dir, i, package, package_dir)
        