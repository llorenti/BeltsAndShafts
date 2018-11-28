import math


#TO-DO:
#       input
#       creation and addition of child nodes in build function
#       search function
#       any other functions I find

# Test stuff for push


class Shaft():
    # class that describes a shaft

    def __init__(self, coordinates = [0,0,0,N]):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.R = coordinates[2]
        self.rotateDir = coordinates[3]


class Node():
    # This is the node class for the tree

    def __init__(self, index = 0, length = 0, meetAng = None, line = None, parent = None):
        self.index = index
        self.R = shafts[index].R
        self.center = (shafts[index].x, shafts[index].y)
        self.spin = shafts[index].rotateDir
        self.length = length
        self.meetAngle = meetAng
        self.line = line
        self.parent = parent
        self.children = []

        
        

    def build(self, remainingShafts = None):

        for i in remainingShafts:
            c2 = (shafts[i].x, shafts[i].y)
            r2 = shafts[i].R
            spin2 = shafts[i].rotateDir

            d = math.sqrt((c2[0] - self.center[0])**2 + (c2[1] - self.center[1])**2)

            if spin2 == self.spin:
                l = math.sqrt( d**2 - (self.R - r2)**2 )
                if l > maxL:
                    continue
                offset = math.acos((self.R - r2)/d)
                if self.spin == 'C':
                    relAngleA = offset
                    relAngleB = offset
                else:
                    relAngleA = 2*math.pi - offset
                    relAngleB = 2*math.pi - offset
            else:
                l = math.sqrt(d**2 - (self.R + r2)**2)
                if l > maxL:
                    continue
                offset = math.acos((self.R + r2)/d)
                if self.spin == 'C':
                    relAngleA = offset
                    relAngleB = math.pi + offset
                else:
                    relAngleA = 2*math.pi - offset
                    relAngleB = math.pi - offset

            thetaRef = math.asin((c2[1] - self.center[1])/d)
            if c2[0] - self.center[0] < 0:
                thetaRef = math.pi - thetaRef
            elif c2[1] - self.center[1] < 0:
                thetaRef = thetaRef + 2*math.pi
            thisLeaveAngle = (thetaRef + relAngleA)%(2*math.pi)
            nextMeetAngle = (thetaRef + relAngleB)%(2*math.pi)

            if self.index != startIndex:
                if self.spin == 'CC':
                    angleDiff = thisLeaveAngle - self.meetAngle
                else:
                    angleDiff = self.meetAngle - thisLeaveAngle
                if angleDiff < 0:
                    angleDiff += 2*math.pi
                if angleDiff > 3*math.pi/2:
                    continue

                arcLength = self.R * angleDiff
                l = l + arcLength

            x1 = self.R * math.cos(thisLeaveAngle) + self.center[0]
            y1 = self.R * math.sin(thisLeaveAngle) + self.center[1]
            x2 = r2 * math.cos(nextMeetAngle) + c2[0]
            y2 = r2 * math.sin(nextMeetAngle) + c2[1]

            if x1 == x2:
                nextLine = ('V', y1, y2, x1)
            else:
                m = (y2 - y1)/(x2 - x1)
                b = y1 - m*x1
                nextLine = (m, b, x1, x2)

            isLineCrossing = findLineCrossing(nextLine)
            if isLineCrossing:
                continue

            isShaftCrossing = findShaftCrossing(nextLine, i)
            if isShaftCrossing:
                continue



    def shortestPath(self):
        # tree searching function



    def findLineCrossing(self, newLine = None, prevNode = self):
        # Recursive function to find line crossings
        # Calls itself with ancestor.parent until crossing is found or root node is reached
        # Stops recursing when crossing is found or when prevNode is root node
        #
        # returns True if crossing found, False if no crossing found

        # MUST include the special case for vertical lines
        # All cases:
        #   two non-vertical lines
        #   one vertical and one non-vertical
        #   two parallel lines -> automatically false

        if prevNode.index == startIndex:
            return False
        elif newLine[0] == 'V' and prevNode.line[0] == 'V':
            return findLineCrossing(newLine, prevNode.parent)
        elif newLine[0] == 'V':
            xc = newLine[3]
            if prevNode.line[2] <= xc and prevNode.line[3] >= xc:
                return True
            else:
                return findLineCrossing(newLine, prevNode.parent)
        elif prevNode.line[0] == 'V':
            xc = prevNode.line[3]
            if newLine[2] <= xc and newLine[3] >= xc:
                return True
            else:
                return findLineCrossing(newLine, prevNode.parent)
        elif newLine[0] == prevNode.line[0]:
            return findLineCrossing(newLine, prevNode.parent)
        else:
            if newLine[3] < prevNode.line[2] or newLine[2] > prevNode.line[3]:
                return findLineCrossing(newLine, prevNode.parent)
            else:
                minX = max(newLine[2], prevNode.line[2])
                maxX = min(newLine[3], prevNode.line[3])
                if ((newLine[0] - prevNode.line[0])*minX + newLine[1] - prevNode.line[1])*
                ((newLine[0] - prevNode.line[0])*maxX + newLine[1] - prevNode.line[1]) < 0:
                    return True
                else:
                    return findLineCrossing(newLine, prevNode.parent)
        


    def findShaftCrossing(self, newLine = None, nextShaftInd = None):
        # Iterative function to find if line crosses shaft

        for (shaft in shafts):
            if shaft.index != self.index and shaft.index != nextShaftInd:

                if newLine[0] == 'V':
                    d = abs(shaft.x - newLine[3])
                else:    
                    x0 = (shaft.x/newLine[0] + shaft.y - newLine[1])/(newLine[0] + 1/newLine[0])
                    d = math.sqrt((x0 - shaft.x)**2 + (newLine[0]*x0 + newLine[1] - shaft.y)**2)
                if d <= shaft.R:
                    return True

        return False        

        
#global constants
numShafts
shafts = []
startIndex 
endIndex
maxL

"get input"
inFile = open("input.txt")


