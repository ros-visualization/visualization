parser grammar InterfaceParser;

options {
	language = Python;
//	output = AST;
	
	tokenVocab = InterfaceLexer;
}

@header {
import sys
import traceback

from InterfaceLexer import InterfaceLexer

class Method(object):
	def __init__(self, name, return_fields, fields, attributes):
		self.name = name
		self.return_fields = return_fields
		self.fields = fields
		self.attributes = attributes
		
		if (self.fields is None):
			self.fields = []
		if (self.return_fields is None):
			self.return_fields = []
		if (self.attributes is None):
			self.attributes = []
		
	def __repr__(self):
		return "\%s: ret: \%s fields: \%s attr: \%s"\%(self.name, self.return_fields, self.fields, self.attributes)

class Interface(object):
	def __init__(self, name):
		self.name = name
		self.methods = []
		
	def __repr__(self):
		return "\%s: \%s"\%(self.name, self.methods)
}

@main {
def main(argv, otherArg=None):
  char_stream = ANTLRFileStream(sys.argv[1])
  lexer = InterfaceLexer(char_stream)
  tokens = CommonTokenStream(lexer)
  parser = InterfaceParser(tokens);

  try:
      parser.main()
  except RecognitionException:
	traceback.print_stack()
}

/*------------------------------------------------------------------
 * PARSER RULES
 *------------------------------------------------------------------*/

@members
{
	interfaces = []
}


method returns [name, rs, ps, attrs]
	: 
	{ 
		$rs = [] 
		$ps = [] 
		$attrs = []
	} 

	(
		attr = ATTRIBUTE
		{
			$attrs.append(attr.text)
		}
	)* 
	rvals = return_vals {$rs = $rvals.rs} my_name=ID {$name = my_name.text} OPENPAREN 
	pvals = parameter_list 
	{ 
		$ps = $pvals.params 
	} 
	CLOSEPAREN SEMICOLON 
	;
	
interface 
	: INTERFACE name=ID { self.interfaces.append(Interface(name.text)) } OPENBRACKET 
		(
		mt = method 
		{
			self.interfaces[-1].methods.append(Method(mt.name, mt.rs, mt.ps, mt.attrs))
		}
		)*
	  	CLOSEBRACKET
	;
	
	
parameter_list returns [params]
	: 
	| tv = type i = ID {$params = [($tv.text, $i.text)]} (COMMA tvo = type io = ID { $params.append(($tvo.text, $io.text)) })* 
	;
	
package_type returns [t]
	: pkg=ID SLASH id=ID {$t = '\%s/\%s'\%(pkg, id) }
	;
	
type returns [type_name]
	: n = ID {$type_name = n}
	| np = package_type {$type_name = np}
	;
	
return_vals returns [rs]
	: 
	| OPENPAREN pl = parameter_list { $rs = pl } CLOSEPAREN 
	;

message 
	: MESSAGE ID OPENBRACKET (type ID SEMICOLON)* CLOSEBRACKET 
	;

main 
	: (interface | message)* EOF
	;

