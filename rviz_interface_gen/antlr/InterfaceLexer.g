lexer grammar InterfaceLexer;

options {
	language = Python;
}

/*------------------------------------------------------------------
 * LEXER RULES
 *------------------------------------------------------------------*/

WHITESPACE : ( '\t' | ' ' | '\r' | '\n'| '\u000C' )+ 	{ $channel = HIDDEN; } ;

ATTRIBUTE : ('sync' | 'async') ;

OPENBRACKET :	 '{';
CLOSEBRACKET :	 '}';
OPENPAREN :	 '(';
CLOSEPAREN :	 ')';
SEMICOLON :	 ';';
COMMA :	 ',';
SLASH :	'/';
INTERFACE :	'interface';
MESSAGE :	'message';

fragment DIGIT : ('0'..'9') ;
fragment LETTER : ('a'..'z'|'A'..'Z') ;
ID  :	(LETTER | '_') (LETTER | DIGIT | '_')* ;

COMMENT
    :   '//' ~('\n'|'\r')* '\r'? '\n' {$channel=HIDDEN;}
    |   '/*' ( options {greedy=false;} : . )* '*/' {$channel=HIDDEN;}
    ;
