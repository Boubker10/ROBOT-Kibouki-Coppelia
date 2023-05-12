import turtle 
import sys
import os
if os.environ.get('DISPLAY','') == '':
    print('no display found. Using :0')
    os.environ.__setitem__('DISPLAY', ':0.0')
t = turtle.Turtle()

t.screen.bgcolor('black')
t.pensize(2)
t.color('green')
t.left(96)
t.backward(188)
t.speed(508)
t.shape('turtle')
def tree(i):
    if i <10:
        return 
    else:
        t.forward(i)
        t.color('orange')
        t.circle(2)
        t.color('brown')
        t.left(38) 
        tree(3*i/4) 
        t.right(60)
        tree(3*i/4)
        t.left(38)
        t.backward(i)
tree(188)
turtle.done()
