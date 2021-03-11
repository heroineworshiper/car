# functions for working with freecad sketches

import FreeCAD as App
import FreeCADGui as Gui
import Sketcher
import Part
import math


doc = App.activeDocument()
gui = Gui.activeDocument()


def toRad(angle):
    return angle * math.pi * 2.0 / 360.0

def toDeg(x):
    return x * 360.0 / 2 / math.pi


# add vector with length & angle to start 
def addLine(start, angle, length):
    result = App.Vector(start.x + length * math.cos(angle),
        start.y + length * math.sin(angle),
        start.z)
    return result



def angleChange(old_angle, new_angle):
    result = new_angle - old_angle
    if result > math.pi: 
        result -= math.pi * 2
    elif result < -math.pi:
        result += math.pi * 2
    return result


def getObj(name):
    for i in doc.Objects:
        if i.Label == name:
            return i
    return None


# make a closed polygon
def makePoly(sketch, coords, closeIt = True):
    prevLine = None
    for i in range(0, len(coords) - 1):
        sketch.addGeometry(Part.LineSegment(coords[i],
            coords[i + 1]),
            False)

    if closeIt:
        sketch.addGeometry(Part.LineSegment(coords[len(coords) - 1],
            coords[0]),
            False)



def vectorsEqual(a, b):
    return a.sub(b).Length < 0.001

def vectorsNotEqual(a, b):
    return a.sub(b).Length >= 0.001

def getSegments(ref, closed):
    # get the line segments
    global starts
    global ends
    global total
    global sorted

    starts = []
    ends = []
    total = ref.GeometryCount
    print('GeometryCount=', total)


    for i in range(0, total):
        start = ref.getPoint(i, 1)
        end = ref.getPoint(i, 2)
        print('line ', i, '=', start, ' -> ', end)
        starts.append(start)
        ends.append(end)


    # sort the line segments
    sorted = []
    taken = []
    for i in range(0, total):
        taken.append(False)

    # find line with 1 unconnected point
    for i in range(0, total):
        gotStart = False
        gotEnd = False
        for j in range(0, total):
            if j != i:
                if vectorsEqual(starts[i], starts[j]) or \
                    vectorsEqual(starts[i], ends[j]):
                    gotStart = True
                if vectorsEqual(ends[i], starts[j]) or \
                    vectorsEqual(ends[i], ends[j]):
                    gotEnd = True

        # nothing connected to the start or end of this line
        if not gotStart and not gotEnd:
            print('line ', i, ' is not connected to anything')
        elif not gotStart or not gotEnd:
            print('line ', i, ' gotStart=', gotStart, ' gotEnd=', gotEnd)
            sorted.append(i)
            taken[i] = True
            break

    if len(sorted) == 0:
        if closed:
            # closed sketch has no starting line
            sorted.append(0)
            taken[0] = True
        else:
            print('*** Got no starting line')
    else:
        print('starting line=', sorted[0])

    # get rest of lines
    prev = sorted[0]
    for i in range(1, total):
        for j in range(0, total):
            if not taken[j]:
                if vectorsEqual(starts[prev], starts[j]) or \
                    vectorsEqual(starts[prev], ends[j]) or \
                    vectorsEqual(ends[prev], starts[j]) or \
                    vectorsEqual(ends[prev], ends[j]):
                    #print('prev=', prev, ' j=', j)
                    taken[j] = True
                    sorted.append(j)
                    prev = j
                    break

    for i in range(0, total):
        print('sorted[', i, '] = ', sorted[i])

    # swap line ends
    for i in range(1, total):
        index1 = sorted[i - 1]
        index2 = sorted[i]
        if i == 1:
            if vectorsNotEqual(ends[index1], starts[index2]) and \
                vectorsNotEqual(ends[index1], ends[index2]):
                temp = starts[index1]
                starts[index1] = ends[index1]
                ends[index1] = temp
        if vectorsNotEqual(ends[index1], starts[index2]):
            temp = starts[index2]
            starts[index2] = ends[index2]
            ends[index2] = temp

    for i in range(0, total):
        print('line ', i, '=', starts[sorted[i]], ' -> ', ends[sorted[i]])





# border for a closed polygon
def makeBorder(sketchTitle, \
    borderW, \
    insideDirection, \
    closed):
    ref = getObj(sketchTitle)
    getSegments(ref, closed)

    # create border lines
    borderLine1 = []
    borderLine2 = []

    for i in range(0, total):
        start = starts[sorted[i]]
        end = ends[sorted[i]]

        if i == 0:
            if closed:
                prevStart = starts[sorted[total - 1]]
                prevEnd = ends[sorted[total - 1]]
                prevAngle = math.atan2(prevEnd.y - prevStart.y, prevEnd.x - prevStart.x)
                nextAngle = math.atan2(end.y - start.y, end.x - start.x)
                angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
                mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            else:
                prevAngle = math.atan2(end.y - start.y, end.x - start.x)
                angle2 = prevAngle + insideDirection
                mag = 1

            borderLine1.append(start)
            borderLine2.append(addLine(start, angle2, mag * borderW))
        if i == total - 1:
            if closed:
                start2 = starts[sorted[0]]
                end2 = ends[sorted[0]]
                prevAngle = math.atan2(end.y - start.y, end.x - start.x)
                nextAngle = math.atan2(end2.y - start2.y, end2.x - start2.x)
                angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
                mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            else:
                nextAngle = math.atan2(end.y - start.y, end.x - start.x)
                angle2 = nextAngle + insideDirection
                mag = 1


            borderLine1.append(end)
            borderLine2.append(addLine(end, angle2, mag * borderW))
        else:
            start2 = starts[sorted[i + 1]]
            end2 = ends[sorted[i + 1]]
            prevAngle = math.atan2(end.y - start.y, end.x - start.x)
            nextAngle = math.atan2(end2.y - start2.y, end2.x - start2.x)
            angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
            mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            borderLine1.append(end)
            borderLine2.append(addLine(end, angle2, mag * borderW))

    border = doc.addObject('Sketcher::SketchObject', sketchTitle)

    for i in range(1, total + 1):
        i1 = i - 1
        i2 = i

        border.addGeometry(Part.LineSegment(borderLine1[i1],
                    borderLine1[i2]),
                    False)
        border.addGeometry(Part.LineSegment(borderLine2[i1],
                    borderLine2[i2]),
                    False)


    if not closed:
# open ended polygon requires closing the ends
        border.addGeometry(Part.LineSegment(borderLine1[0],
                    borderLine2[0]),
                    False)
        border.addGeometry(Part.LineSegment(borderLine1[total],
                    borderLine2[total]),
                    False)

    return border
