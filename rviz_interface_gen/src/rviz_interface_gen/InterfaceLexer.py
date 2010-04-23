# $ANTLR 3.1.2 /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g 2010-04-22 17:11:19

import sys
from antlr3 import *
from antlr3.compat import set, frozenset


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


class InterfaceLexer(Lexer):

    grammarFileName = "/wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g"
    antlr_version = version_str_to_tuple("3.1.2")
    antlr_version_str = "3.1.2"

    def __init__(self, input=None, state=None):
        if state is None:
            state = RecognizerSharedState()
        Lexer.__init__(self, input, state)

        self.dfa8 = self.DFA8(
            self, 8,
            eot = self.DFA8_eot,
            eof = self.DFA8_eof,
            min = self.DFA8_min,
            max = self.DFA8_max,
            accept = self.DFA8_accept,
            special = self.DFA8_special,
            transition = self.DFA8_transition
            )






    # $ANTLR start "WHITESPACE"
    def mWHITESPACE(self, ):

        try:
            _type = WHITESPACE
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:11:12: ( ( '\\t' | ' ' | '\\r' | '\\n' | '\\u000C' )+ )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:11:14: ( '\\t' | ' ' | '\\r' | '\\n' | '\\u000C' )+
            pass 
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:11:14: ( '\\t' | ' ' | '\\r' | '\\n' | '\\u000C' )+
            cnt1 = 0
            while True: #loop1
                alt1 = 2
                LA1_0 = self.input.LA(1)

                if ((9 <= LA1_0 <= 10) or (12 <= LA1_0 <= 13) or LA1_0 == 32) :
                    alt1 = 1


                if alt1 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:
                    pass 
                    if (9 <= self.input.LA(1) <= 10) or (12 <= self.input.LA(1) <= 13) or self.input.LA(1) == 32:
                        self.input.consume()
                    else:
                        mse = MismatchedSetException(None, self.input)
                        self.recover(mse)
                        raise mse



                else:
                    if cnt1 >= 1:
                        break #loop1

                    eee = EarlyExitException(1, self.input)
                    raise eee

                cnt1 += 1


            #action start
            _channel = HIDDEN; 
            #action end



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "WHITESPACE"



    # $ANTLR start "ATTRIBUTE"
    def mATTRIBUTE(self, ):

        try:
            _type = ATTRIBUTE
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:13:11: ( ( 'sync' | 'async' ) )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:13:13: ( 'sync' | 'async' )
            pass 
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:13:13: ( 'sync' | 'async' )
            alt2 = 2
            LA2_0 = self.input.LA(1)

            if (LA2_0 == 115) :
                alt2 = 1
            elif (LA2_0 == 97) :
                alt2 = 2
            else:
                nvae = NoViableAltException("", 2, 0, self.input)

                raise nvae

            if alt2 == 1:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:13:14: 'sync'
                pass 
                self.match("sync")


            elif alt2 == 2:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:13:23: 'async'
                pass 
                self.match("async")






            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "ATTRIBUTE"



    # $ANTLR start "OPENBRACKET"
    def mOPENBRACKET(self, ):

        try:
            _type = OPENBRACKET
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:15:13: ( '{' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:15:16: '{'
            pass 
            self.match(123)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "OPENBRACKET"



    # $ANTLR start "CLOSEBRACKET"
    def mCLOSEBRACKET(self, ):

        try:
            _type = CLOSEBRACKET
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:16:14: ( '}' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:16:17: '}'
            pass 
            self.match(125)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "CLOSEBRACKET"



    # $ANTLR start "OPENPAREN"
    def mOPENPAREN(self, ):

        try:
            _type = OPENPAREN
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:17:11: ( '(' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:17:14: '('
            pass 
            self.match(40)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "OPENPAREN"



    # $ANTLR start "CLOSEPAREN"
    def mCLOSEPAREN(self, ):

        try:
            _type = CLOSEPAREN
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:18:12: ( ')' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:18:15: ')'
            pass 
            self.match(41)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "CLOSEPAREN"



    # $ANTLR start "SEMICOLON"
    def mSEMICOLON(self, ):

        try:
            _type = SEMICOLON
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:19:11: ( ';' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:19:14: ';'
            pass 
            self.match(59)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "SEMICOLON"



    # $ANTLR start "COMMA"
    def mCOMMA(self, ):

        try:
            _type = COMMA
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:20:7: ( ',' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:20:10: ','
            pass 
            self.match(44)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "COMMA"



    # $ANTLR start "SLASH"
    def mSLASH(self, ):

        try:
            _type = SLASH
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:21:7: ( '/' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:21:9: '/'
            pass 
            self.match(47)



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "SLASH"



    # $ANTLR start "INTERFACE"
    def mINTERFACE(self, ):

        try:
            _type = INTERFACE
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:22:11: ( 'interface' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:22:13: 'interface'
            pass 
            self.match("interface")



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "INTERFACE"



    # $ANTLR start "MESSAGE"
    def mMESSAGE(self, ):

        try:
            _type = MESSAGE
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:23:9: ( 'message' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:23:11: 'message'
            pass 
            self.match("message")



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "MESSAGE"



    # $ANTLR start "RETURNS"
    def mRETURNS(self, ):

        try:
            _type = RETURNS
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:24:9: ( 'returns' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:24:11: 'returns'
            pass 
            self.match("returns")



            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "RETURNS"



    # $ANTLR start "DIGIT"
    def mDIGIT(self, ):

        try:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:26:16: ( ( '0' .. '9' ) )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:26:18: ( '0' .. '9' )
            pass 
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:26:18: ( '0' .. '9' )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:26:19: '0' .. '9'
            pass 
            self.matchRange(48, 57)







        finally:

            pass

    # $ANTLR end "DIGIT"



    # $ANTLR start "LETTER"
    def mLETTER(self, ):

        try:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:27:17: ( ( 'a' .. 'z' | 'A' .. 'Z' ) )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:27:19: ( 'a' .. 'z' | 'A' .. 'Z' )
            pass 
            if (65 <= self.input.LA(1) <= 90) or (97 <= self.input.LA(1) <= 122):
                self.input.consume()
            else:
                mse = MismatchedSetException(None, self.input)
                self.recover(mse)
                raise mse





        finally:

            pass

    # $ANTLR end "LETTER"



    # $ANTLR start "ID"
    def mID(self, ):

        try:
            _type = ID
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:28:5: ( ( LETTER | '_' ) ( LETTER | DIGIT | '_' )* )
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:28:7: ( LETTER | '_' ) ( LETTER | DIGIT | '_' )*
            pass 
            if (65 <= self.input.LA(1) <= 90) or self.input.LA(1) == 95 or (97 <= self.input.LA(1) <= 122):
                self.input.consume()
            else:
                mse = MismatchedSetException(None, self.input)
                self.recover(mse)
                raise mse

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:28:22: ( LETTER | DIGIT | '_' )*
            while True: #loop3
                alt3 = 4
                LA3 = self.input.LA(1)
                if LA3 == 65 or LA3 == 66 or LA3 == 67 or LA3 == 68 or LA3 == 69 or LA3 == 70 or LA3 == 71 or LA3 == 72 or LA3 == 73 or LA3 == 74 or LA3 == 75 or LA3 == 76 or LA3 == 77 or LA3 == 78 or LA3 == 79 or LA3 == 80 or LA3 == 81 or LA3 == 82 or LA3 == 83 or LA3 == 84 or LA3 == 85 or LA3 == 86 or LA3 == 87 or LA3 == 88 or LA3 == 89 or LA3 == 90 or LA3 == 97 or LA3 == 98 or LA3 == 99 or LA3 == 100 or LA3 == 101 or LA3 == 102 or LA3 == 103 or LA3 == 104 or LA3 == 105 or LA3 == 106 or LA3 == 107 or LA3 == 108 or LA3 == 109 or LA3 == 110 or LA3 == 111 or LA3 == 112 or LA3 == 113 or LA3 == 114 or LA3 == 115 or LA3 == 116 or LA3 == 117 or LA3 == 118 or LA3 == 119 or LA3 == 120 or LA3 == 121 or LA3 == 122:
                    alt3 = 1
                elif LA3 == 48 or LA3 == 49 or LA3 == 50 or LA3 == 51 or LA3 == 52 or LA3 == 53 or LA3 == 54 or LA3 == 55 or LA3 == 56 or LA3 == 57:
                    alt3 = 2
                elif LA3 == 95:
                    alt3 = 3

                if alt3 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:28:23: LETTER
                    pass 
                    self.mLETTER()


                elif alt3 == 2:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:28:32: DIGIT
                    pass 
                    self.mDIGIT()


                elif alt3 == 3:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:28:40: '_'
                    pass 
                    self.match(95)


                else:
                    break #loop3





            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "ID"



    # $ANTLR start "COMMENT"
    def mCOMMENT(self, ):

        try:
            _type = COMMENT
            _channel = DEFAULT_CHANNEL

            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:31:5: ( '//' (~ ( '\\n' | '\\r' ) )* ( '\\r' )? '\\n' | '/*' ( options {greedy=false; } : . )* '*/' )
            alt7 = 2
            LA7_0 = self.input.LA(1)

            if (LA7_0 == 47) :
                LA7_1 = self.input.LA(2)

                if (LA7_1 == 47) :
                    alt7 = 1
                elif (LA7_1 == 42) :
                    alt7 = 2
                else:
                    nvae = NoViableAltException("", 7, 1, self.input)

                    raise nvae

            else:
                nvae = NoViableAltException("", 7, 0, self.input)

                raise nvae

            if alt7 == 1:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:31:9: '//' (~ ( '\\n' | '\\r' ) )* ( '\\r' )? '\\n'
                pass 
                self.match("//")
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:31:14: (~ ( '\\n' | '\\r' ) )*
                while True: #loop4
                    alt4 = 2
                    LA4_0 = self.input.LA(1)

                    if ((0 <= LA4_0 <= 9) or (11 <= LA4_0 <= 12) or (14 <= LA4_0 <= 65535)) :
                        alt4 = 1


                    if alt4 == 1:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:31:14: ~ ( '\\n' | '\\r' )
                        pass 
                        if (0 <= self.input.LA(1) <= 9) or (11 <= self.input.LA(1) <= 12) or (14 <= self.input.LA(1) <= 65535):
                            self.input.consume()
                        else:
                            mse = MismatchedSetException(None, self.input)
                            self.recover(mse)
                            raise mse



                    else:
                        break #loop4


                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:31:28: ( '\\r' )?
                alt5 = 2
                LA5_0 = self.input.LA(1)

                if (LA5_0 == 13) :
                    alt5 = 1
                if alt5 == 1:
                    # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:31:28: '\\r'
                    pass 
                    self.match(13)



                self.match(10)
                #action start
                _channel=HIDDEN;
                #action end


            elif alt7 == 2:
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:32:9: '/*' ( options {greedy=false; } : . )* '*/'
                pass 
                self.match("/*")
                # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:32:14: ( options {greedy=false; } : . )*
                while True: #loop6
                    alt6 = 2
                    LA6_0 = self.input.LA(1)

                    if (LA6_0 == 42) :
                        LA6_1 = self.input.LA(2)

                        if (LA6_1 == 47) :
                            alt6 = 2
                        elif ((0 <= LA6_1 <= 46) or (48 <= LA6_1 <= 65535)) :
                            alt6 = 1


                    elif ((0 <= LA6_0 <= 41) or (43 <= LA6_0 <= 65535)) :
                        alt6 = 1


                    if alt6 == 1:
                        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:32:42: .
                        pass 
                        self.matchAny()


                    else:
                        break #loop6


                self.match("*/")
                #action start
                _channel=HIDDEN;
                #action end


            self._state.type = _type
            self._state.channel = _channel

        finally:

            pass

    # $ANTLR end "COMMENT"



    def mTokens(self):
        # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:8: ( WHITESPACE | ATTRIBUTE | OPENBRACKET | CLOSEBRACKET | OPENPAREN | CLOSEPAREN | SEMICOLON | COMMA | SLASH | INTERFACE | MESSAGE | RETURNS | ID | COMMENT )
        alt8 = 14
        alt8 = self.dfa8.predict(self.input)
        if alt8 == 1:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:10: WHITESPACE
            pass 
            self.mWHITESPACE()


        elif alt8 == 2:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:21: ATTRIBUTE
            pass 
            self.mATTRIBUTE()


        elif alt8 == 3:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:31: OPENBRACKET
            pass 
            self.mOPENBRACKET()


        elif alt8 == 4:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:43: CLOSEBRACKET
            pass 
            self.mCLOSEBRACKET()


        elif alt8 == 5:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:56: OPENPAREN
            pass 
            self.mOPENPAREN()


        elif alt8 == 6:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:66: CLOSEPAREN
            pass 
            self.mCLOSEPAREN()


        elif alt8 == 7:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:77: SEMICOLON
            pass 
            self.mSEMICOLON()


        elif alt8 == 8:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:87: COMMA
            pass 
            self.mCOMMA()


        elif alt8 == 9:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:93: SLASH
            pass 
            self.mSLASH()


        elif alt8 == 10:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:99: INTERFACE
            pass 
            self.mINTERFACE()


        elif alt8 == 11:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:109: MESSAGE
            pass 
            self.mMESSAGE()


        elif alt8 == 12:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:117: RETURNS
            pass 
            self.mRETURNS()


        elif alt8 == 13:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:125: ID
            pass 
            self.mID()


        elif alt8 == 14:
            # /wg/bgr/jfaust/ros/dev/visualization-2.0/visualization/rviz_interface_gen/antlr/InterfaceLexer.g:1:128: COMMENT
            pass 
            self.mCOMMENT()







    # lookup tables for DFA #8

    DFA8_eot = DFA.unpack(
        u"\2\uffff\2\16\6\uffff\1\22\3\16\1\uffff\2\16\2\uffff\10\16\1\40"
        u"\4\16\1\uffff\1\40\7\16\1\54\1\55\1\16\2\uffff\1\57\1\uffff"
        )

    DFA8_eof = DFA.unpack(
        u"\60\uffff"
        )

    DFA8_min = DFA.unpack(
        u"\1\11\1\uffff\1\171\1\163\6\uffff\1\52\1\156\2\145\1\uffff\1\156"
        u"\1\171\2\uffff\1\164\1\163\1\164\1\143\1\156\1\145\1\163\1\165"
        u"\1\60\1\143\1\162\1\141\1\162\1\uffff\1\60\1\146\1\147\1\156\1"
        u"\141\1\145\1\163\1\143\2\60\1\145\2\uffff\1\60\1\uffff"
        )

    DFA8_max = DFA.unpack(
        u"\1\175\1\uffff\1\171\1\163\6\uffff\1\57\1\156\2\145\1\uffff\1\156"
        u"\1\171\2\uffff\1\164\1\163\1\164\1\143\1\156\1\145\1\163\1\165"
        u"\1\172\1\143\1\162\1\141\1\162\1\uffff\1\172\1\146\1\147\1\156"
        u"\1\141\1\145\1\163\1\143\2\172\1\145\2\uffff\1\172\1\uffff"
        )

    DFA8_accept = DFA.unpack(
        u"\1\uffff\1\1\2\uffff\1\3\1\4\1\5\1\6\1\7\1\10\4\uffff\1\15\2\uffff"
        u"\1\16\1\11\15\uffff\1\2\13\uffff\1\13\1\14\1\uffff\1\12"
        )

    DFA8_special = DFA.unpack(
        u"\60\uffff"
        )

            
    DFA8_transition = [
        DFA.unpack(u"\2\1\1\uffff\2\1\22\uffff\1\1\7\uffff\1\6\1\7\2\uffff"
        u"\1\11\2\uffff\1\12\13\uffff\1\10\5\uffff\32\16\4\uffff\1\16\1\uffff"
        u"\1\3\7\16\1\13\3\16\1\14\4\16\1\15\1\2\7\16\1\4\1\uffff\1\5"),
        DFA.unpack(u""),
        DFA.unpack(u"\1\17"),
        DFA.unpack(u"\1\20"),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u"\1\21\4\uffff\1\21"),
        DFA.unpack(u"\1\23"),
        DFA.unpack(u"\1\24"),
        DFA.unpack(u"\1\25"),
        DFA.unpack(u""),
        DFA.unpack(u"\1\26"),
        DFA.unpack(u"\1\27"),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u"\1\30"),
        DFA.unpack(u"\1\31"),
        DFA.unpack(u"\1\32"),
        DFA.unpack(u"\1\33"),
        DFA.unpack(u"\1\34"),
        DFA.unpack(u"\1\35"),
        DFA.unpack(u"\1\36"),
        DFA.unpack(u"\1\37"),
        DFA.unpack(u"\12\16\7\uffff\32\16\4\uffff\1\16\1\uffff\32\16"),
        DFA.unpack(u"\1\41"),
        DFA.unpack(u"\1\42"),
        DFA.unpack(u"\1\43"),
        DFA.unpack(u"\1\44"),
        DFA.unpack(u""),
        DFA.unpack(u"\12\16\7\uffff\32\16\4\uffff\1\16\1\uffff\32\16"),
        DFA.unpack(u"\1\45"),
        DFA.unpack(u"\1\46"),
        DFA.unpack(u"\1\47"),
        DFA.unpack(u"\1\50"),
        DFA.unpack(u"\1\51"),
        DFA.unpack(u"\1\52"),
        DFA.unpack(u"\1\53"),
        DFA.unpack(u"\12\16\7\uffff\32\16\4\uffff\1\16\1\uffff\32\16"),
        DFA.unpack(u"\12\16\7\uffff\32\16\4\uffff\1\16\1\uffff\32\16"),
        DFA.unpack(u"\1\56"),
        DFA.unpack(u""),
        DFA.unpack(u""),
        DFA.unpack(u"\12\16\7\uffff\32\16\4\uffff\1\16\1\uffff\32\16"),
        DFA.unpack(u"")
    ]

    # class definition for DFA #8

    DFA8 = DFA
 



def main(argv, stdin=sys.stdin, stdout=sys.stdout, stderr=sys.stderr):
    from antlr3.main import LexerMain
    main = LexerMain(InterfaceLexer)
    main.stdin = stdin
    main.stdout = stdout
    main.stderr = stderr
    main.execute(argv)


if __name__ == '__main__':
    main(sys.argv)
