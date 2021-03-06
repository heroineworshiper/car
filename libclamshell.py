# create a clamshell enclosure from a sketch

import FreeCAD as App
import FreeCADGui as Gui
import Sketcher
import Part
import math

doc = App.activeDocument()
gui = Gui.activeDocument()

# wall dimensions
OVERLAP_W = 1.0
OVERLAP_GAP = 0.1
OVERLAP_H = 2.0
TOTAL_W = 2.0
SKETCH_Y = 20.0



# make beveling tool instead of walls
MAKE_BEVEL = True

# bevel dimensions
BEVEL_W1 = 3.0
BEVEL_W2 = 1.0
BEVEL_HMARGIN = 5.0


def getObj(name):
    for i in doc.Objects:
        if i.Label == name:
            return i
    return None


def toDeg(x):
    return x * 360.0 / 2 / math.pi

# make a closed polygon
def makePoly(sketch, coords):
    for i in range(0, len(coords) - 1):
        sketch.addGeometry(Part.LineSegment(coords[i],
            coords[i + 1]),
            False)
    sketch.addGeometry(Part.LineSegment(coords[len(coords) - 1],
        coords[0]),
        False)


def vectorsEqual(a, b):
    return a.sub(b).Length < 0.0001

def vectorsNotEqual(a, b):
    return a.sub(b).Length >= 0.0001

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

def extruderDefaults(x):
    x.DirMode = "Normal"
    x.DirLink = None
    x.LengthFwd = 0.0
    x.LengthRev = 0.0
    x.Solid = True
    x.Reversed = False
    x.Symmetric = False
    x.TaperAngle = 0.000000000000000
    x.TaperAngleRev = 0.000000000000000




def getSegments(ref, closed):
    # get the line segments
    global starts
    global ends
    global total
    global sorted

    starts = []
    ends = []
    total = ref.GeometryCount
    for i in range(0, total):
        start = ref.getPoint(i, 1)
        end = ref.getPoint(i, 2)
    #    print('line ', i, '=', start, ' -> ', end)
        starts.append(start)
        ends.append(end)


    # sort the line segments
    sorted = []
    taken = []
    for i in range(0, total):
        taken.append(False)

    # find line with 1 dangling point
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
        if not gotStart or not gotEnd:
            sorted.append(i)
            taken[i] = True
            break

    #print('sorted=', len(sorted))
    if len(sorted) == 0:
        if closed:
            # closed sketch has no starting line
            sorted.append(0)
            taken[0] = True
        else:
            print('*** Got no starting line')

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

    #for i in range(0, total):
    #    print('sorted[', i, '] = ', sorted[i])

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



def createDiagonalSketch(insideDirection):
    diagonals = doc.addObject('Sketcher::SketchObject','diagonals')
    gui.getObject(diagonals.Label).Visibility = False
    diagonals.Placement = App.Placement(
        App.Vector(0.0, 0.0, SKETCH_Y),
        App.Rotation(App.Vector(0, 0, 1), 0.0))
    newPolys = 0
    for i in range(0, total):
        start = starts[sorted[i]]
        end = ends[sorted[i]]
        angle = math.atan2(end.y - start.y, end.x - start.x)
# test for right angle
        if abs(angleChange(angle, math.pi / 2)) > 0.001 and \
            abs(angleChange(angle, math.pi)) > 0.001 and \
            abs(angleChange(angle, math.pi * 3 / 2)) > 0.001 and \
            abs(angleChange(angle, 0.0)) > 0.001:
# create diagonal polygon
            coords = []
            angle2 = angle + insideDirection
            coords.append(start)
            coords.append(end)
            coords.append(addLine(end, angle2, TOTAL_W + 1.0))
            coords.append(addLine(start, angle2, TOTAL_W + 1.0))
            makePoly(diagonals, coords)
            newPolys += 1

    if newPolys == 0:
        doc.removeObject(diagonals.Label)
        diagonals = None
    return diagonals



def makeClam(ref, wall_h, y0, insideDirection, closed):
    getSegments(ref, closed)

    # create wall lines
    outer_line1 = []
    outer_line2 = []
    inner_line1 = []
    inner_line2 = []
    wall_line1 = []
    wall_line2 = []

    # DEBUG
    #total = 4

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

            wall_line1.append(start)
            wall_line2.append(addLine(start, angle2, mag * TOTAL_W))
            outer_line1.append(start)
            outer_line2.append(addLine(start, angle2, mag * OVERLAP_W))
            inner_line1.append(addLine(start, angle2, mag * (OVERLAP_W + OVERLAP_GAP)))
            inner_line2.append(addLine(start, angle2, mag * (OVERLAP_W + OVERLAP_GAP + OVERLAP_W)))
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
            wall_line1.append(end)
            wall_line2.append(addLine(end, angle2, mag * TOTAL_W))
            outer_line1.append(end)
            outer_line2.append(addLine(end, angle2, mag * OVERLAP_W))
            inner_line1.append(addLine(end, angle2, mag * (OVERLAP_W + OVERLAP_GAP)))
            inner_line2.append(addLine(end, angle2, mag * (OVERLAP_W + OVERLAP_GAP + OVERLAP_W)))
        else:
            start2 = starts[sorted[i + 1]]
            end2 = ends[sorted[i + 1]]
            prevAngle = math.atan2(end.y - start.y, end.x - start.x)
            nextAngle = math.atan2(end2.y - start2.y, end2.x - start2.x)
            angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
            mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            print(' prevAngle=', \
                toDeg(prevAngle), \
                'nextAngle=', \
                toDeg(nextAngle), \
                ' angle2=', \
                toDeg(angle2))
            wall_line1.append(end)
            wall_line2.append(addLine(end, angle2, mag * (TOTAL_W)))
            outer_line1.append(end)
            outer_line2.append(addLine(end, angle2, mag * (OVERLAP_W)))
            inner_line1.append(addLine(end, angle2, mag * (OVERLAP_W + OVERLAP_GAP)))
            inner_line2.append(addLine(end, angle2, mag * (OVERLAP_W + OVERLAP_GAP + OVERLAP_W)))


    if True:
# create wall sketches
        outer = doc.addObject('Sketcher::SketchObject','outer')
        gui.getObject(outer.Label).Visibility = False
        outer.Placement = App.Placement(
            App.Vector(0.0, 0.0, SKETCH_Y),
            App.Rotation(App.Vector(0, 0, 1), 0.0))
        inner = doc.addObject('Sketcher::SketchObject','inner')
        gui.getObject(inner.Label).Visibility = False
        inner.Placement = App.Placement(
            App.Vector(0.0, 0.0, SKETCH_Y),
            App.Rotation(App.Vector(0, 0, 1), 0.0))
        topWallSketch = doc.addObject('Sketcher::SketchObject','wall')
        gui.getObject(topWallSketch.Label).Visibility = False
        topWallSketch.Placement = App.Placement(
            App.Vector(0.0, 0.0, SKETCH_Y),
            App.Rotation(App.Vector(0, 0, 1), 0.0))

        for i in range(1, total + 1):
            #print('addGeometry i=', i, ' ', wall_line1[i - 1], ' -> ', wall_line1[i])
            i1 = i - 1
            i2 = i

            topWallSketch.addGeometry(Part.LineSegment(wall_line1[i1],
                        wall_line1[i2]),
                        False)
            topWallSketch.addGeometry(Part.LineSegment(wall_line2[i1],
                        wall_line2[i2]),
                        False)
            outer.addGeometry(Part.LineSegment(outer_line1[i1],
                        outer_line1[i2]),
                        False)
            outer.addGeometry(Part.LineSegment(outer_line2[i1],
                        outer_line2[i2]),
                        False)
            inner.addGeometry(Part.LineSegment(inner_line1[i1],
                        inner_line1[i2]),
                        False)
            inner.addGeometry(Part.LineSegment(inner_line2[i1],
                        inner_line2[i2]),
                        False)


        if not closed:
    # open ended clamshell requires closing the ends
            topWallSketch.addGeometry(Part.LineSegment(wall_line1[0],
                        wall_line2[0]),
                        False)
            outer.addGeometry(Part.LineSegment(outer_line1[0],
                        outer_line2[0]),
                        False)
            inner.addGeometry(Part.LineSegment(inner_line1[0],
                        inner_line2[0]),
                        False)
            topWallSketch.addGeometry(Part.LineSegment(wall_line1[total],
                        wall_line2[total]),
                        False)
            outer.addGeometry(Part.LineSegment(outer_line1[total],
                        outer_line2[total]),
                        False)
            inner.addGeometry(Part.LineSegment(inner_line1[total],
                        inner_line2[total]),
                        False)

# don't reuse the sketch for the bottom
        bottomWallSketch = doc.copyObject(topWallSketch, True)
        bottomWallSketch.Placement = App.Placement(
            App.Vector(0.0, 0.0, SKETCH_Y),
            App.Rotation(App.Vector(0, 0, 1), 0.0))

    # extrude them
        bottomWall = doc.addObject('Part::Extrusion', 'bottomWall')
        extruderDefaults(bottomWall)
        bottomWall.Placement = App.Placement(
            App.Vector(0.0, 0.0, y0 - SKETCH_Y),
            App.Rotation(App.Vector(0, 0, 1), 0.0))
        bottomWall.Base = bottomWallSketch
        bottomWall.LengthFwd = wall_h - OVERLAP_H / 2


        bottomOuter = doc.addObject('Part::Extrusion', 'bottomOuter')
        extruderDefaults(bottomOuter)
        bottomOuter.Placement = App.Placement(
            App.Vector(0.0, 0.0, y0 + wall_h - SKETCH_Y - OVERLAP_H / 2),
            App.Rotation(App.Vector(0, 0, 1), 0.0))
        bottomOuter.Base = outer
        bottomOuter.LengthFwd = 2.0



        topWall = doc.addObject('Part::Extrusion', 'topWall')
        extruderDefaults(topWall)
        topWall.Placement = App.Placement(
            App.Vector(0.0, 0.0, y0 + wall_h * 2 - SKETCH_Y),
            App.Rotation(App.Vector(0, 0, 1), 0.0))
        topWall.Base = topWallSketch
        topWall.LengthRev = wall_h - OVERLAP_H / 2


        topInner = doc.addObject('Part::Extrusion', 'topInner')
        extruderDefaults(topInner)
        topInner.Placement = App.Placement(
            App.Vector(0.0, 0.0, y0 + wall_h - SKETCH_Y - OVERLAP_H),
            App.Rotation(App.Vector(0, 0, 1), 0.0))
        topInner.Base = inner
        topInner.LengthFwd = OVERLAP_H + 2.0


    if False:
# create diagonal fillers for the isogrids
# we now use a border routine for this
        diagonals = createDiagonalSketch(insideDirection)

        if diagonals is not None:
# extrude the diagonals
            topDiagonals = doc.addObject('Part::Extrusion', 'topDiagonals')
            extruderDefaults(topDiagonals)
            topDiagonals.Placement = App.Placement(
                App.Vector(0.0, 0.0, y0 + wall_h * 2 + OVERLAP_H - SKETCH_Y),
                App.Rotation(App.Vector(0, 0, 1), 0.0))
            topDiagonals.Base = diagonals
            topDiagonals.LengthFwd = 2.0

# create a different sketch for bottom
        diagonals = createDiagonalSketch(insideDirection)
        if diagonals is not None:
            bottomDiagonals = doc.addObject('Part::Extrusion', 'bottomDiagonals')
            extruderDefaults(bottomDiagonals)
            bottomDiagonals.Placement = App.Placement(
                App.Vector(0.0, 0.0, y0 - SKETCH_Y),
                App.Rotation(App.Vector(0, 0, 1), 0.0))
            bottomDiagonals.Base = diagonals
            bottomDiagonals.LengthRev = 2.0





def makeBevel(ref, inner_z, outer_z, wall_w, insideDirection, closed):
    getSegments(ref, closed)

# outer width to get a 45 degree angle
    outer_w = abs(inner_z - outer_z) - wall_w
    

    bevel_inner1 = []
#    bevel_inner2 = []
    bevel_outer1 = []
#    bevel_outer2 = []

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

            bevel_inner1.append(addLine(start, angle2, mag * wall_w))
# excluding margin from the mag multiply is required to get the right angle
#            bevel_inner2.append(addLine(start, angle2, mag * (-BEVEL_W2) - BEVEL_HMARGIN))
            bevel_outer1.append(addLine(start, angle2, mag * -outer_w))
#            bevel_outer2.append(addLine(start, angle2, mag * (-BEVEL_W2) - BEVEL_HMARGIN))

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

            bevel_inner1.append(addLine(end, angle2, mag * wall_w))
#            bevel_inner2.append(addLine(end, angle2, mag * (-BEVEL_W2) - BEVEL_HMARGIN))
            bevel_outer1.append(addLine(end, angle2, mag * -outer_w))
#            bevel_outer2.append(addLine(end, angle2, mag * (-BEVEL_W2) - BEVEL_HMARGIN))
        else:
            start2 = starts[sorted[i + 1]]
            end2 = ends[sorted[i + 1]]
            prevAngle = math.atan2(end.y - start.y, end.x - start.x)
            nextAngle = math.atan2(end2.y - start2.y, end2.x - start2.x)
            angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
            mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))


            bevel_inner1.append(addLine(end, angle2, mag * wall_w))
#            bevel_inner2.append(addLine(end, angle2, mag * (-BEVEL_W2) - BEVEL_HMARGIN))
            bevel_outer1.append(addLine(end, angle2, mag * -outer_w))
#            bevel_outer2.append(addLine(end, angle2, mag * (-BEVEL_W2) - BEVEL_HMARGIN))

    bevelInner = doc.addObject('Sketcher::SketchObject','bevelInner')
    gui.getObject(bevelInner.Label).Visibility = False
    bevelInner.Placement = App.Placement(
        App.Vector(0.0, 0.0, inner_z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))
    bevelOuter = doc.addObject('Sketcher::SketchObject','bevelOuter')
    gui.getObject(bevelOuter.Label).Visibility = False
    bevelOuter.Placement = App.Placement(
        App.Vector(0.0, 0.0, outer_z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))

# can't do a loft of a hollow tube, so make a union of a solid tube
    for i in range(1, total + 1):
        i1 = i - 1
        i2 = i
        bevelInner.addGeometry(Part.LineSegment(bevel_inner1[i1],
                    bevel_inner1[i2]),
                    False)
#        bevelInner.addGeometry(Part.LineSegment(bevel_inner2[i1],
#                    bevel_inner2[i2]),
#                    False)
        bevelOuter.addGeometry(Part.LineSegment(bevel_outer1[i1],
                    bevel_outer1[i2]),
                    False)
#        bevelOuter.addGeometry(Part.LineSegment(bevel_outer2[i1],
#                    bevel_outer2[i2]),
#                    False)

    if not closed:
        bevelInner.addGeometry(Part.LineSegment(bevel_inner1[0],
                    bevel_inner1[total]),
                    False)
        bevelOuter.addGeometry(Part.LineSegment(bevel_outer1[0],
                    bevel_outer1[total]),
                    False)
#        bevelInner.addGeometry(Part.LineSegment(bevel_inner1[0],
#                    bevel_inner2[0]),
#                    False)
#        bevelOuter.addGeometry(Part.LineSegment(bevel_outer1[0],
#                    bevel_outer2[0]),
#                    False)
#        bevelInner.addGeometry(Part.LineSegment(bevel_inner1[total],
#                    bevel_inner2[total]),
#                    False)
#        bevelOuter.addGeometry(Part.LineSegment(bevel_outer1[total],
#                    bevel_outer2[total]),
#                    False)

    loft = doc.addObject('Part::Loft','bevelTool')
    loft.Sections = [bevelInner, bevelOuter]
    loft.Solid=True
    loft.Ruled=False
    loft.Closed=False
    loft.Placement = App.Placement(
        App.Vector(0.0, 0.0, 0.0),
        App.Rotation(App.Vector(0, 0, 1), 0.0))



doc = App.activeDocument()
gui = Gui.activeDocument()












