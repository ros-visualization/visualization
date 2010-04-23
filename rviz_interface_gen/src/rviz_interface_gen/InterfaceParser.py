# $ANTLR 3.1.2 /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g 2010-04-22 17:11:19

import sys
from antlr3 import *
from antlr3.compat import set, frozenset
         
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
		return "%s: ret: %s fields: %s attr: %s"%(self.name, self.return_fields, self.fields, self.attributes)

class Interface(object):
	def __init__(self, name):
		self.name = name
		self.methods = []
		
	def __repr__(self):
		return "%s: %s"%(self.name, self.methods)



# for convenience in actions
HIDDEN = BaseRecognizer.HIDDEN

# token types
CLOSEPAREN=9
LETTER=17
ATTRIBUTE=5
OPENBRACKET=6
WHITESPACE=4
SEMICOLON=10
ID=18
EOF=-1
CLOSEBRACKET=7
SLASH=12
COMMA=11
MESSAGE=14
INTERFACE=13
DIGIT=16
OPENPAREN=8
COMMENT=19
RETURNS=15

# token names
tokenNames = [
    "<invalid>", "<EOR>", "<DOWN>", "<UP>", 
    "WHITESPACE", "ATTRIBUTE", "OPENBRACKET", "CLOSEBRACKET", "OPENPAREN", 
    "CLOSEPAREN", "SEMICOLON", "COMMA", "SLASH", "INTERFACE", "MESSAGE", 
    "RETURNS", "DIGIT", "LETTER", "ID", "COMMENT"
]




class InterfaceParser(Parser):
    grammarFileName = "/wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g"
    antlr_version = version_str_to_tuple("3.1.2")
    antlr_version_str = "3.1.2"
    tokenNames = tokenNames

    def __init__(self, input, state=None):
        if state is None:
            state = RecognizerSharedState()

        Parser.__init__(self, input, state)







                


        

     
    interfaces = []


    class method_return(ParserRuleReturnScope):
        def __init__(self):
            ParserRuleReturnScope.__init__(self)

            self.name = None
            self.rs = None
            self.ps = None
            self.attrs = None




    # $ANTLR start "method"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:65:1: method returns [name, rs, ps, attrs] : (attr= ATTRIBUTE )* my_name= ID OPENPAREN pvals= parameter_list CLOSEPAREN ( RETURNS rvals= return_vals )? SEMICOLON ;
    def method(self, ):

        retval = self.method_return()
        retval.start = self.input.LT(1)

        attr = None
        my_name = None
        pvals = None

        rvals = None


        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:66:2: ( (attr= ATTRIBUTE )* my_name= ID OPENPAREN pvals= parameter_list CLOSEPAREN ( RETURNS rvals= return_vals )? SEMICOLON )
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:67:2: (attr= ATTRIBUTE )* my_name= ID OPENPAREN pvals= parameter_list CLOSEPAREN ( RETURNS rvals= return_vals )? SEMICOLON
                pass 
                #action start
                   
                retval.rs = [] 
                retval.ps = [] 
                retval.attrs = []
                	
                #action end
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:73:2: (attr= ATTRIBUTE )*
                while True: #loop1
                    alt1 = 2
                    LA1_0 = self.input.LA(1)

                    if (LA1_0 == ATTRIBUTE) :
                        alt1 = 1


                    if alt1 == 1:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:74:3: attr= ATTRIBUTE
                        pass 
                        attr=self.match(self.input, ATTRIBUTE, self.FOLLOW_ATTRIBUTE_in_method78)
                        #action start
                           
                        retval.attrs.append(attr.text)
                        		
                        #action end


                    else:
                        break #loop1


                my_name=self.match(self.input, ID, self.FOLLOW_ID_in_method92)
                #action start
                retval.name = my_name.text
                #action end
                self.match(self.input, OPENPAREN, self.FOLLOW_OPENPAREN_in_method96)
                self._state.following.append(self.FOLLOW_parameter_list_in_method104)
                pvals = self.parameter_list()

                self._state.following.pop()
                #action start
                   
                retval.ps = pvals 
                	
                #action end
                self.match(self.input, CLOSEPAREN, self.FOLLOW_CLOSEPAREN_in_method112)
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:84:13: ( RETURNS rvals= return_vals )?
                alt2 = 2
                LA2_0 = self.input.LA(1)

                if (LA2_0 == RETURNS) :
                    alt2 = 1
                if alt2 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:84:14: RETURNS rvals= return_vals
                    pass 
                    self.match(self.input, RETURNS, self.FOLLOW_RETURNS_in_method115)
                    self._state.following.append(self.FOLLOW_return_vals_in_method121)
                    rvals = self.return_vals()

                    self._state.following.pop()
                    #action start
                    retval.rs = rvals
                    #action end



                self.match(self.input, SEMICOLON, self.FOLLOW_SEMICOLON_in_method127)



                retval.stop = self.input.LT(-1)


            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return retval

    # $ANTLR end "method"


    # $ANTLR start "interface"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:87:1: interface : INTERFACE name= ID OPENBRACKET (mt= method )* CLOSEBRACKET ;
    def interface(self, ):

        name = None
        mt = None


        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:88:2: ( INTERFACE name= ID OPENBRACKET (mt= method )* CLOSEBRACKET )
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:88:4: INTERFACE name= ID OPENBRACKET (mt= method )* CLOSEBRACKET
                pass 
                self.match(self.input, INTERFACE, self.FOLLOW_INTERFACE_in_interface141)
                name=self.match(self.input, ID, self.FOLLOW_ID_in_interface145)
                #action start
                self.interfaces.append(Interface(name.text)) 
                #action end
                self.match(self.input, OPENBRACKET, self.FOLLOW_OPENBRACKET_in_interface149)
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:89:3: (mt= method )*
                while True: #loop3
                    alt3 = 2
                    LA3_0 = self.input.LA(1)

                    if (LA3_0 == ATTRIBUTE or LA3_0 == ID) :
                        alt3 = 1


                    if alt3 == 1:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:90:3: mt= method
                        pass 
                        self._state.following.append(self.FOLLOW_method_in_interface162)
                        mt = self.method()

                        self._state.following.pop()
                        #action start
                           
                        self.interfaces[-1].methods.append(Method(mt.name, mt.rs, mt.ps, mt.attrs))
                        		
                        #action end


                    else:
                        break #loop3


                self.match(self.input, CLOSEBRACKET, self.FOLLOW_CLOSEBRACKET_in_interface178)




            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return 

    # $ANTLR end "interface"


    # $ANTLR start "parameter_list"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:99:1: parameter_list returns [params] : ( | tv= type i= ID ( COMMA tvo= type io= ID )* );
    def parameter_list(self, ):

        params = None

        i = None
        io = None
        tv = None

        tvo = None


        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:100:2: ( | tv= type i= ID ( COMMA tvo= type io= ID )* )
                alt5 = 2
                LA5_0 = self.input.LA(1)

                if (LA5_0 == CLOSEPAREN) :
                    alt5 = 1
                elif (LA5_0 == ID) :
                    alt5 = 2
                else:
                    nvae = NoViableAltException("", 5, 0, self.input)

                    raise nvae

                if alt5 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:101:2: 
                    pass 

                elif alt5 == 2:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:101:4: tv= type i= ID ( COMMA tvo= type io= ID )*
                    pass 
                    self._state.following.append(self.FOLLOW_type_in_parameter_list204)
                    tv = self.type()

                    self._state.following.pop()
                    i=self.match(self.input, ID, self.FOLLOW_ID_in_parameter_list210)
                    #action start
                    params = [(((tv is not None) and [self.input.toString(tv.start,tv.stop)] or [None])[0], i.text)]
                    #action end
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:101:55: ( COMMA tvo= type io= ID )*
                    while True: #loop4
                        alt4 = 2
                        LA4_0 = self.input.LA(1)

                        if (LA4_0 == COMMA) :
                            alt4 = 1


                        if alt4 == 1:
                            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:101:56: COMMA tvo= type io= ID
                            pass 
                            self.match(self.input, COMMA, self.FOLLOW_COMMA_in_parameter_list215)
                            self._state.following.append(self.FOLLOW_type_in_parameter_list221)
                            tvo = self.type()

                            self._state.following.pop()
                            io=self.match(self.input, ID, self.FOLLOW_ID_in_parameter_list227)
                            #action start
                            params.append((((tvo is not None) and [self.input.toString(tvo.start,tvo.stop)] or [None])[0], io.text)) 
                            #action end


                        else:
                            break #loop4





            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return params

    # $ANTLR end "parameter_list"


    # $ANTLR start "package_type"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:104:1: package_type returns [t] : pkg= ID SLASH id= ID ;
    def package_type(self, ):

        t = None

        pkg = None
        id = None

        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:105:2: (pkg= ID SLASH id= ID )
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:105:4: pkg= ID SLASH id= ID
                pass 
                pkg=self.match(self.input, ID, self.FOLLOW_ID_in_package_type250)
                self.match(self.input, SLASH, self.FOLLOW_SLASH_in_package_type252)
                id=self.match(self.input, ID, self.FOLLOW_ID_in_package_type256)
                #action start
                t = '%s/%s'%(pkg, id) 
                #action end




            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return t

    # $ANTLR end "package_type"

    class type_return(ParserRuleReturnScope):
        def __init__(self):
            ParserRuleReturnScope.__init__(self)

            self.type_name = None




    # $ANTLR start "type"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:108:1: type returns [type_name] : (n= ID | np= package_type );
    def type(self, ):

        retval = self.type_return()
        retval.start = self.input.LT(1)

        n = None
        np = None


        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:109:2: (n= ID | np= package_type )
                alt6 = 2
                LA6_0 = self.input.LA(1)

                if (LA6_0 == ID) :
                    LA6_1 = self.input.LA(2)

                    if (LA6_1 == SLASH) :
                        alt6 = 2
                    elif (LA6_1 == ID) :
                        alt6 = 1
                    else:
                        nvae = NoViableAltException("", 6, 1, self.input)

                        raise nvae

                else:
                    nvae = NoViableAltException("", 6, 0, self.input)

                    raise nvae

                if alt6 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:109:4: n= ID
                    pass 
                    n=self.match(self.input, ID, self.FOLLOW_ID_in_type278)
                    #action start
                    retval.type_name = n
                    #action end


                elif alt6 == 2:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:110:4: np= package_type
                    pass 
                    self._state.following.append(self.FOLLOW_package_type_in_type289)
                    np = self.package_type()

                    self._state.following.pop()
                    #action start
                    retval.type_name = np
                    #action end


                retval.stop = self.input.LT(-1)


            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return retval

    # $ANTLR end "type"


    # $ANTLR start "return_vals"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:113:1: return_vals returns [rs] : ( | OPENPAREN pl= parameter_list CLOSEPAREN );
    def return_vals(self, ):

        rs = None

        pl = None


        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:114:2: ( | OPENPAREN pl= parameter_list CLOSEPAREN )
                alt7 = 2
                LA7_0 = self.input.LA(1)

                if (LA7_0 == SEMICOLON) :
                    alt7 = 1
                elif (LA7_0 == OPENPAREN) :
                    alt7 = 2
                else:
                    nvae = NoViableAltException("", 7, 0, self.input)

                    raise nvae

                if alt7 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:115:2: 
                    pass 

                elif alt7 == 2:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:115:4: OPENPAREN pl= parameter_list CLOSEPAREN
                    pass 
                    self.match(self.input, OPENPAREN, self.FOLLOW_OPENPAREN_in_return_vals311)
                    self._state.following.append(self.FOLLOW_parameter_list_in_return_vals317)
                    pl = self.parameter_list()

                    self._state.following.pop()
                    #action start
                    rs = pl 
                    #action end
                    self.match(self.input, CLOSEPAREN, self.FOLLOW_CLOSEPAREN_in_return_vals321)



            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return rs

    # $ANTLR end "return_vals"


    # $ANTLR start "message"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:118:1: message : MESSAGE ID OPENBRACKET ( type ID SEMICOLON )* CLOSEBRACKET ;
    def message(self, ):

        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:119:2: ( MESSAGE ID OPENBRACKET ( type ID SEMICOLON )* CLOSEBRACKET )
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:119:4: MESSAGE ID OPENBRACKET ( type ID SEMICOLON )* CLOSEBRACKET
                pass 
                self.match(self.input, MESSAGE, self.FOLLOW_MESSAGE_in_message334)
                self.match(self.input, ID, self.FOLLOW_ID_in_message336)
                self.match(self.input, OPENBRACKET, self.FOLLOW_OPENBRACKET_in_message338)
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:119:27: ( type ID SEMICOLON )*
                while True: #loop8
                    alt8 = 2
                    LA8_0 = self.input.LA(1)

                    if (LA8_0 == ID) :
                        alt8 = 1


                    if alt8 == 1:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:119:28: type ID SEMICOLON
                        pass 
                        self._state.following.append(self.FOLLOW_type_in_message341)
                        self.type()

                        self._state.following.pop()
                        self.match(self.input, ID, self.FOLLOW_ID_in_message343)
                        self.match(self.input, SEMICOLON, self.FOLLOW_SEMICOLON_in_message345)


                    else:
                        break #loop8


                self.match(self.input, CLOSEBRACKET, self.FOLLOW_CLOSEBRACKET_in_message349)




            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return 

    # $ANTLR end "message"


    # $ANTLR start "main"
    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:122:1: main : ( interface | message )* EOF ;
    def main(self, ):

        try:
            try:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:123:2: ( ( interface | message )* EOF )
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:123:4: ( interface | message )* EOF
                pass 
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:123:4: ( interface | message )*
                while True: #loop9
                    alt9 = 3
                    LA9_0 = self.input.LA(1)

                    if (LA9_0 == INTERFACE) :
                        alt9 = 1
                    elif (LA9_0 == MESSAGE) :
                        alt9 = 2


                    if alt9 == 1:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:123:5: interface
                        pass 
                        self._state.following.append(self.FOLLOW_interface_in_main363)
                        self.interface()

                        self._state.following.pop()


                    elif alt9 == 2:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceParser.g:123:17: message
                        pass 
                        self._state.following.append(self.FOLLOW_message_in_main367)
                        self.message()

                        self._state.following.pop()


                    else:
                        break #loop9


                self.match(self.input, EOF, self.FOLLOW_EOF_in_main371)




            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)
        finally:

            pass

        return 

    # $ANTLR end "main"


    # Delegated rules


 

    FOLLOW_ATTRIBUTE_in_method78 = frozenset([5, 18])
    FOLLOW_ID_in_method92 = frozenset([8])
    FOLLOW_OPENPAREN_in_method96 = frozenset([9, 18])
    FOLLOW_parameter_list_in_method104 = frozenset([9])
    FOLLOW_CLOSEPAREN_in_method112 = frozenset([10, 15])
    FOLLOW_RETURNS_in_method115 = frozenset([8, 10])
    FOLLOW_return_vals_in_method121 = frozenset([10])
    FOLLOW_SEMICOLON_in_method127 = frozenset([1])
    FOLLOW_INTERFACE_in_interface141 = frozenset([18])
    FOLLOW_ID_in_interface145 = frozenset([6])
    FOLLOW_OPENBRACKET_in_interface149 = frozenset([5, 7, 18])
    FOLLOW_method_in_interface162 = frozenset([5, 7, 18])
    FOLLOW_CLOSEBRACKET_in_interface178 = frozenset([1])
    FOLLOW_type_in_parameter_list204 = frozenset([18])
    FOLLOW_ID_in_parameter_list210 = frozenset([1, 11])
    FOLLOW_COMMA_in_parameter_list215 = frozenset([18])
    FOLLOW_type_in_parameter_list221 = frozenset([18])
    FOLLOW_ID_in_parameter_list227 = frozenset([1, 11])
    FOLLOW_ID_in_package_type250 = frozenset([12])
    FOLLOW_SLASH_in_package_type252 = frozenset([18])
    FOLLOW_ID_in_package_type256 = frozenset([1])
    FOLLOW_ID_in_type278 = frozenset([1])
    FOLLOW_package_type_in_type289 = frozenset([1])
    FOLLOW_OPENPAREN_in_return_vals311 = frozenset([9, 18])
    FOLLOW_parameter_list_in_return_vals317 = frozenset([9])
    FOLLOW_CLOSEPAREN_in_return_vals321 = frozenset([1])
    FOLLOW_MESSAGE_in_message334 = frozenset([18])
    FOLLOW_ID_in_message336 = frozenset([6])
    FOLLOW_OPENBRACKET_in_message338 = frozenset([7, 18])
    FOLLOW_type_in_message341 = frozenset([18])
    FOLLOW_ID_in_message343 = frozenset([10])
    FOLLOW_SEMICOLON_in_message345 = frozenset([7, 18])
    FOLLOW_CLOSEBRACKET_in_message349 = frozenset([1])
    FOLLOW_interface_in_main363 = frozenset([13, 14])
    FOLLOW_message_in_main367 = frozenset([13, 14])
    FOLLOW_EOF_in_main371 = frozenset([1])



       
def main(argv, otherArg=None):
  char_stream = ANTLRFileStream(sys.argv[1])
  lexer = InterfaceLexer(char_stream)
  tokens = CommonTokenStream(lexer)
  parser = InterfaceParser(tokens);

  try:
      parser.main()
  except RecognitionException:
	traceback.print_stack()


if __name__ == '__main__':
    main(sys.argv)
