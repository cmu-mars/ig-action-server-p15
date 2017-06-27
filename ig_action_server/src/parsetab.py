
# parsetab.py
# This file is automatically generated. Do not edit.
_tabversion = '3.8'

_lr_method = 'LALR'

_lr_signature = 'E717584F30781CC9B7300FF43045AB70'
    
_lr_action_items = {'LPAR':([1,5,22,23,24,25,26,27,28,29,30,31,32,33,34,35,38,39,],[3,7,41,42,43,44,45,46,47,48,49,50,51,52,53,54,57,58,]),'SETLOCALIZATIONFIDELITY':([18,],[22,]),'CONS':([9,37,],[12,-4,]),'MOVEABS':([18,],[24,]),'DO':([14,],[18,]),'MOVE':([18,],[25,]),'STOP':([20,56,],[38,38,]),'RPAR':([8,10,15,16,19,21,60,64,68,69,73,74,77,100,101,102,104,106,107,108,120,122,123,129,133,],[-2,13,-3,-9,37,-8,79,83,87,88,92,-5,95,112,113,114,116,-6,118,-7,125,127,128,131,134,]),'MOVEABSH':([18,],[23,]),'DEADLINE':([18,],[26,]),'TURNABS':([18,],[28,]),'STRING':([47,50,58,94,],[66,69,77,107,]),'MOVETO':([18,],[29,]),'RECALIBRATE':([18,],[30,]),'TURNREL':([18,],[27,]),'VISIBLE':([20,56,],[39,39,]),'SAY':([18,],[31,]),'NUM':([7,17,41,42,43,44,45,46,48,49,51,52,53,54,55,57,59,80,81,82,84,85,86,89,90,91,93,96,109,110,111,115,117,124,126,132,],[11,21,60,61,62,63,64,65,67,68,70,71,72,73,74,76,78,97,98,99,100,101,102,103,104,105,106,108,119,120,121,122,123,129,130,133,]),'COMMA':([4,11,37,61,62,63,65,66,67,70,71,72,76,97,98,99,103,105,119,121,130,],[6,14,-4,80,81,82,84,85,86,89,90,91,94,109,110,111,115,117,124,126,132,]),'THEN':([36,40,75,79,83,87,88,92,95,112,113,114,116,118,125,127,128,131,134,],[55,59,93,-21,-23,-20,-11,-19,-24,-17,-16,-12,-18,-25,-14,-13,-15,-22,-10,]),'$end':([2,13,],[0,-1,]),'LOCATE':([18,],[32,]),'END':([14,],[16,]),'GOTO':([14,],[17,]),'ELSE':([78,],[96,]),'P':([0,],[1,]),'V':([3,6,12,],[5,5,5,]),'FORWARD':([18,],[33,]),'UNTIL':([36,79,83,87,88,92,112,113,114,116,125,127,128,131,134,],[56,-21,-23,-20,-11,-19,-17,-16,-12,-18,-14,-13,-15,-22,-10,]),'IF':([14,],[20,]),'MOVEREL':([18,],[34,]),'NIL':([6,12,],[8,8,]),'CHARGE':([18,],[35,]),}

_lr_action = {}
for _k, _v in _lr_action_items.items():
   for _x,_y in zip(_v[0],_v[1]):
      if not _x in _lr_action:  _lr_action[_x] = {}
      _lr_action[_x][_k] = _y
del _lr_action_items

_lr_goto_items = {'vertex':([3,6,12,],[4,9,9,]),'vertices':([6,12,],[10,15,]),'content':([14,],[19,]),'cnd':([20,56,],[40,75,]),'program':([0,],[2,]),'action':([18,],[36,]),}

_lr_goto = {}
for _k, _v in _lr_goto_items.items():
   for _x, _y in zip(_v[0], _v[1]):
       if not _x in _lr_goto: _lr_goto[_x] = {}
       _lr_goto[_x][_k] = _y
del _lr_goto_items
_lr_productions = [
  ("S' -> program","S'",1,None,None,None),
  ('program -> P LPAR vertex COMMA vertices RPAR','program',6,'p_program','parserIG.py',53),
  ('vertices -> NIL','vertices',1,'p_vertices','parserIG.py',57),
  ('vertices -> vertex CONS vertices','vertices',3,'p_vertices','parserIG.py',58),
  ('vertex -> V LPAR NUM COMMA content RPAR','vertex',6,'p_vertex','parserIG.py',65),
  ('content -> DO action THEN NUM','content',4,'p_content','parserIG.py',71),
  ('content -> DO action UNTIL cnd THEN NUM','content',6,'p_content','parserIG.py',72),
  ('content -> IF cnd THEN NUM ELSE NUM','content',6,'p_content','parserIG.py',73),
  ('content -> GOTO NUM','content',2,'p_content','parserIG.py',74),
  ('content -> END','content',1,'p_content','parserIG.py',75),
  ('action -> MOVE LPAR NUM COMMA NUM COMMA NUM COMMA NUM COMMA NUM RPAR','action',12,'p_action','parserIG.py',98),
  ('action -> SAY LPAR STRING RPAR','action',4,'p_action','parserIG.py',99),
  ('action -> MOVETO LPAR NUM COMMA NUM RPAR','action',6,'p_action','parserIG.py',100),
  ('action -> LOCATE LPAR NUM COMMA NUM COMMA NUM RPAR','action',8,'p_action','parserIG.py',101),
  ('action -> MOVEABS LPAR NUM COMMA NUM COMMA NUM RPAR','action',8,'p_action','parserIG.py',102),
  ('action -> MOVEREL LPAR NUM COMMA NUM COMMA NUM RPAR','action',8,'p_action','parserIG.py',103),
  ('action -> TURNABS LPAR STRING COMMA NUM RPAR','action',6,'p_action','parserIG.py',104),
  ('action -> TURNREL LPAR NUM COMMA NUM RPAR','action',6,'p_action','parserIG.py',105),
  ('action -> FORWARD LPAR NUM COMMA NUM RPAR','action',6,'p_action','parserIG.py',106),
  ('action -> CHARGE LPAR NUM RPAR','action',4,'p_action','parserIG.py',107),
  ('action -> RECALIBRATE LPAR NUM RPAR','action',4,'p_action','parserIG.py',108),
  ('action -> SETLOCALIZATIONFIDELITY LPAR NUM RPAR','action',4,'p_action','parserIG.py',109),
  ('action -> MOVEABSH LPAR NUM COMMA NUM COMMA NUM COMMA NUM RPAR','action',10,'p_action','parserIG.py',110),
  ('action -> DEADLINE LPAR NUM RPAR','action',4,'p_action','parserIG.py',111),
  ('cnd -> VISIBLE LPAR STRING RPAR','cnd',4,'p_cnd','parserIG.py',143),
  ('cnd -> STOP LPAR NUM COMMA STRING RPAR','cnd',6,'p_cnd','parserIG.py',144),
]
