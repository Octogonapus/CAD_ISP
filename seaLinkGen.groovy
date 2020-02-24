import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.*
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import eu.mihosoft.vrl.v3d.*
import javafx.scene.paint.Color
import javafx.scene.transform.Affine

class MyCadGen implements ICadGenerator {

    private double grid = 25.0
    private double rearMotorBracketWidth = 15.0
    private double rearMotorBracketThickness = 5.0
    private double passiveHingePinRadius = 2.5
    private double passiveHingePinHeight = 5.0
    private double rearShaftBracketWidth = rearMotorBracketWidth
    private double rearShaftBracketThickness = rearMotorBracketThickness
    private double bridgeThickness = 5.0
    private double boltThroughholeRadius = 2.5

    static CSG reverseDHValues(CSG incoming, DHLink dh) {
        println "Reversing " + dh
        TransformNR step = new TransformNR(dh.DhStep(0))
        Transform move = TransformFactory.nrToCSG(step)
        return incoming.transformed(move)
    }

    static CSG moveDHValues(CSG incoming, DHLink dh) {
        TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
        Transform move = TransformFactory.nrToCSG(step)
        return incoming.transformed(move)
    }

    CSG makePassiveHingePin() {
        // Half height because we just want to "overwrite" the corner radius on the bottom half
        CSG bottomHalf = new Cylinder(passiveHingePinRadius, passiveHingePinHeight / 2)
                .toCSG()
                .toZMax()

        CSG topHalf = new RoundedCylinder(passiveHingePinRadius, passiveHingePinHeight)
                .cornerRadius(1.0)
                .toCSG()
                .toZMax()

        CSG passiveHingePin = CSG.unionAll([bottomHalf, topHalf])
        return passiveHingePin
    }

    CSG makeMotorBracket(CSG motorCSG, DHLink link) {
        // Motor bracket thickness is from z=0 to maxZ. Bolt heads will be flush with the top
        //  of the bracket.
        CSG frontMotorMountBracket = new Cube(
                motorCSG.totalX,
                motorCSG.totalY,
                motorCSG.maxZ
        ).toCSG()

        def linkRLength = -motorCSG.minX + bridgeThickness
        CSG linkRFront = new Cube(linkRLength, motorCSG.totalY, motorCSG.maxZ).toCSG().toXMax()
        frontMotorMountBracket = frontMotorMountBracket.union(linkRFront)

        // Line up with the mounting face
        frontMotorMountBracket = frontMotorMountBracket.toZMin()

        // Center it with the motor
        frontMotorMountBracket = frontMotorMountBracket.movex(motorCSG.centerX)
        frontMotorMountBracket = frontMotorMountBracket.movey(motorCSG.centerY)

        // Cut out mounting points
        frontMotorMountBracket = frontMotorMountBracket.difference(motorCSG)

        CSG rearMotorMountBracket = new Cube(
                rearMotorBracketWidth,
                rearMotorBracketWidth,
                rearMotorBracketThickness
        ).toCSG()

        CSG linkRRear = new Cube(linkRLength, rearMotorBracketWidth, rearMotorBracketThickness).toCSG().toXMax()
        rearMotorMountBracket = rearMotorMountBracket.union(linkRRear)

        // Line up with back face
        rearMotorMountBracket = rearMotorMountBracket.toZMax()
        rearMotorMountBracket = rearMotorMountBracket.movez(motorCSG.minZ)

        def frontAndRearBrackets = CSG.unionAll([frontMotorMountBracket, rearMotorMountBracket])
        def bridge = new Cube(bridgeThickness, motorCSG.totalY, frontAndRearBrackets.totalZ).toCSG()
        bridge = bridge.toZMax().movez(frontAndRearBrackets.maxZ)
        bridge = bridge.toXMin().movex(frontAndRearBrackets.minX)
        bridge = bridge.movey(motorCSG.centerY)

        CSG passiveHingePin = makePassiveHingePin()
        // Line up with the back of the rear bracket
        passiveHingePin = passiveHingePin.movez(rearMotorMountBracket.minZ)

        def bracket = CSG.unionAll([frontAndRearBrackets, bridge, passiveHingePin])
        bracket.setColor(Color.BURLYWOOD)
        return bracket
    }

    CSG makeShaftBracket(CSG motorCSG, CSG shaftCSG, CSG shaftCollar, DHLink link) {
        double shaftBracketX = Math.max(shaftCollar.totalX, 15.0)
        double shaftBracketY = Math.max(shaftCollar.totalY, 15.0)

        CSG frontShaftMountBracket = new Cube(
                shaftBracketX,
                shaftBracketY,
                10.0
        ).toCSG()

        def limitedR = Math.sqrt(
                Math.pow(Math.max(motorCSG.maxX, -motorCSG.minX), 2) +
                        Math.pow(Math.max(motorCSG.maxY, -motorCSG.minY), 2)
        ) + bridgeThickness

        CSG linkRFront = new Cube(limitedR, shaftBracketY, 5.0).toCSG().toXMin()
        frontShaftMountBracket = frontShaftMountBracket.union(linkRFront)

        // Line up with the end of the motor and the start of the shaft
        frontShaftMountBracket = frontShaftMountBracket.toZMin()
        frontShaftMountBracket = frontShaftMountBracket.movez(motorCSG.maxZ)

        CSG rearShaftMountBracket = new Cube(
                rearShaftBracketWidth,
                rearShaftBracketWidth,
                rearShaftBracketThickness
        ).toCSG()

        CSG linkRBack = new Cube(limitedR, rearShaftBracketWidth, rearShaftBracketThickness).toCSG().toXMin()
        rearShaftMountBracket = rearShaftMountBracket.union(linkRBack)

        rearShaftMountBracket = rearShaftMountBracket.toZMax()

        // Make a space for the passive hinge pin
        CSG passiveHingePin = makePassiveHingePin()
        rearShaftMountBracket = rearShaftMountBracket.difference(passiveHingePin)
        // Line up centered with the motor body and at the end of the rear motor bracket
        rearShaftMountBracket = rearShaftMountBracket.movez(motorCSG.minZ)
        rearShaftMountBracket = rearShaftMountBracket.movez(-rearMotorBracketThickness)

        def frontAndRearBrackets = CSG.unionAll([frontShaftMountBracket, rearShaftMountBracket])
        def bridge = new Cube(bridgeThickness, motorCSG.totalY, frontAndRearBrackets.totalZ).toCSG()
        bridge = bridge.toZMax().movez(frontAndRearBrackets.maxZ)
        bridge = bridge.toXMax().movex(frontAndRearBrackets.maxX)

        def bracket = CSG.unionAll([frontAndRearBrackets, bridge])
        bracket = bracket.difference(shaftCollar.movez(motorCSG.maxZ))
        bracket.setColor(Color.CYAN)
        return bracket
    }

    @Override
    ArrayList<CSG> generateBody(MobileBase mobileBase) {
        def vitaminLocations = new HashMap<TransformNR, List<String>>()
        def allCad = new ArrayList<CSG>()
        double baseGrid = grid * 2
        double baseBoltThickness = 15
        double baseCoreheight = 1
        String boltsize = "M5x25"
        def thrustBearingSize = "Thrust_1andAHalfinch"
        for (DHParameterKinematics d : mobileBase.getAllDHChains()) {
            // Hardware to engineering units configuration
            LinkConfiguration conf = d.getLinkConfiguration(0)
            // loading the vitamins referenced in the configuration
            TransformNR locationOfMotorMount = d.getRobotToFiducialTransform()
            TransformNR locationOfMotorMountCopy = locationOfMotorMount.copy()
            if (locationOfMotorMount.getZ() > baseCoreheight)
                baseCoreheight = locationOfMotorMount.getZ()
            vitaminLocations.put(locationOfMotorMountCopy, [
                    "ballBearing",
                    thrustBearingSize
            ])
            vitaminLocations.put(locationOfMotorMount, [
                    conf.getElectroMechanicalType(),
                    conf.getElectroMechanicalSize()
            ])
        }
        def insert = ["heatedThreadedInsert", "M5"]
        def insertMeasurements = Vitamins.getConfiguration(insert[0], insert[1])
        def mountLocations = [new TransformNR(baseGrid, baseGrid, 0, new RotationNR(180, 0, 0)),
                              new TransformNR(baseGrid, -baseGrid, 0, new RotationNR(180, 0, 0)),
                              new TransformNR(-baseGrid, baseGrid, 0, new RotationNR(180, 0, 0)),
                              new TransformNR(-baseGrid, -baseGrid, 0, new RotationNR(180, 0, 0))]
        mountLocations.each {
            vitaminLocations.put(it, ["capScrew", boltsize])
            vitaminLocations.put(it.copy().translateZ(insertMeasurements.installLength as double), insert)
        }

        double totalMass = 0
        TransformNR centerOfMassFromCentroid = new TransformNR()
        for (TransformNR tr : vitaminLocations.keySet()) {
            def vitaminType = vitaminLocations.get(tr)[0]
            def vitaminSize = vitaminLocations.get(tr)[1]

            HashMap<String, Object> measurements = Vitamins.getConfiguration(vitaminType, vitaminSize)

            CSG vitaminCad = Vitamins.get(vitaminType, vitaminSize)
            Transform move = TransformFactory.nrToCSG(tr)
            CSG part = vitaminCad.transformed(move)
            part.setManipulator(mobileBase.getRootListener())
            allCad.add(part)

            double massCentroidYValue = measurements.massCentroidY as double
            double massCentroidXValue = measurements.massCentroidX as double
            double massCentroidZValue = measurements.massCentroidZ as double
            double massKgValue = measurements.massKg as double
            println "Base Vitamin " + vitaminType + " " + vitaminSize
            try {
                TransformNR COMCentroid = tr.times(new TransformNR(
                        massCentroidXValue, massCentroidYValue, massCentroidZValue, new RotationNR()
                ))
                totalMass += massKgValue
            } catch (Exception ex) {
                BowlerStudio.printStackTrace(ex)
            }

            //do com calculation here for centerOfMassFromCentroid and totalMass
        }
        //Do additional CAD and add to the running CoM
        def thrustMeasurements = Vitamins.getConfiguration("ballBearing",
                thrustBearingSize)
        CSG baseCore = new Cylinder(
                thrustMeasurements.outerDiameter / 2 + 5,
                baseCoreheight + thrustMeasurements.width / 2
        ).toCSG()
        CSG baseCoreshort = new Cylinder(
                thrustMeasurements.outerDiameter / 2 + 5,
                baseCoreheight * 3.0 / 4.0
        ).toCSG()
        CSG mountLug = new Cylinder(15, baseBoltThickness).toCSG().toZMax()
        CSG mountCap = Parabola.coneByHeight(15, 20)
                .rotx(-90)
                .toZMax()
                .movez(-baseBoltThickness)
        def coreParts = [baseCore]
        mountLocations.each {
            def place = TransformFactory.nrToCSG(it)
            coreParts.add(CSG.hullAll(mountLug.transformed(place), baseCoreshort))
            coreParts.add(mountCap.transformed(place))
        }

        // assemble the base
        CSG wire = new Cube(17, 200, 5).toCSG()
                .toZMin()
                .toYMin()
        CSG vitamin_roundMotor_WPI_gb37y3530_50en = Vitamins.get("roundMotor", "WPI-gb37y3530-50en")
                .toZMin()
                .union(wire)
        allCad.add(vitamin_roundMotor_WPI_gb37y3530_50en)

        def baseCad = CSG.unionAll(coreParts).difference(vitamin_roundMotor_WPI_gb37y3530_50en)
        baseCad.setManipulator(mobileBase.getRootListener())
        allCad.add(baseCad)

        mobileBase.setMassKg(totalMass)
        mobileBase.setCenterOfMassFromCentroid(centerOfMassFromCentroid)

        return allCad
    }

    @Override
    ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
        println("Generating CAD for " + d + " linkIndex=" + linkIndex)

        Map<TransformNR, List<String>> vitaminLocations = new HashMap<TransformNR, List<String>>()
        List<DHLink> dhLinks = d.getChain().getLinks()
        List<CSG> allCad = new ArrayList<CSG>()

        DHLink dh = dhLinks.get(linkIndex)
        // Hardware to engineering units configuration
        LinkConfiguration conf = d.getLinkConfiguration(linkIndex)
        // Engineering units to kinematics link (limits and hardware type abstraction)
        AbstractLink abstractLink = d.getAbstractLink(linkIndex)
        // Transform used by the UI to render the location of the object
        Affine manipulator = dh.getListener()

        TransformNR locationOfMotorMount = new TransformNR(dh.DhStep(0)).inverse()
        def shaftType = conf.getShaftType()
        def shaftSize = conf.getShaftSize()
        def shaftCad = Vitamins.get(shaftType, shaftSize)
        vitaminLocations.put(
                locationOfMotorMount,
                [shaftType, shaftSize]
        )

        def motorType = conf.getElectroMechanicalType()
        def motorSize = conf.getElectroMechanicalSize()
        def motorCad = Vitamins.get(motorType, motorSize)

        if (linkIndex != d.getNumberOfLinks() - 1) {
            // If this is not the last link (the last link does not get a motor)
            // The motor for the first link is part of the base
            LinkConfiguration nextConf = d.getLinkConfiguration(linkIndex + 1)
            DHLink nextDh = dhLinks.get(linkIndex + 1)

            def nextMotorType = nextConf.getElectroMechanicalType()
            def nextMotorSize = nextConf.getElectroMechanicalSize()
            def nextMotorCad = Vitamins.get(nextMotorType, nextMotorSize)
            vitaminLocations.put(new TransformNR(), [nextMotorType, nextMotorSize])

            if (linkIndex != 0) {
                def motorBracket = makeMotorBracket(nextMotorCad, dh)
                CSG shaftCollar
                if (motorType == "hobbyServo") {
                    // The shaft for a servo is the horn, so just difference that
                    shaftCollar = shaftCad
                } else {
                    // Otherwise we need to bolt something onto the shaft to mesh with the link
                    // bracket, so use a shaft collar
                    shaftCollar = Vitamins.get("brushlessBoltOnShaft", "sunnysky_x2204")
                }
                def shaftBracket = makeShaftBracket(motorCad, shaftCad, shaftCollar, dh)

                CSG motorBracketSlice = createNegXSlice(motorBracket)
                CSG connectionMotorBracketMount = createNegXConnectionMount(motorBracketSlice)

                CSG shaftBracketSlice = createPosXSlice(shaftBracket, dh)
                CSG connectionShaftBracketMount = createPosXConnectionMount(shaftBracketSlice)
                shaftBracket = moveDHValues(shaftBracket, dh)
                connectionShaftBracketMount = moveDHValues(connectionShaftBracketMount, dh)

                def connection = connectionMotorBracketMount.hull(connectionShaftBracketMount)

                CSG motorKeepawayCylinder = createMotorKeepawayCylinder(motorCad, dh)
                connection = connection.difference(motorKeepawayCylinder)
                        .difference(motorBracket.hull())
                        .difference(shaftBracket.hull())
                connection.setColor(Color.MEDIUMPURPLE)

                def link = CSG.unionAll([motorBracket, shaftBracket, connection])
                def (CSG posZHalf, CSG negZHalf) = sliceLinkConnection(dh, link, motorCad, nextMotorCad)
                def linkCSGs = [posZHalf, negZHalf]
                linkCSGs.each {
                    it.setManipulator(dh.getListener())
                }
                allCad.addAll(linkCSGs)
            }
        } else if (linkIndex == d.getNumberOfLinks() - 1) {
            CSG shaftCollar
            if (motorType == "hobbyServo") {
                // The shaft for a servo is the horn, so just difference that
                shaftCollar = shaftCad
            } else {
                // Otherwise we need to bolt something onto the shaft to mesh with the link
                // bracket, so use a shaft collar
                shaftCollar = Vitamins.get("brushlessBoltOnShaft", "sunnysky_x2204")
            }
            def shaftBracket = makeShaftBracket(motorCad, shaftCad, shaftCollar, dh)

            CSG shaftBracketSlice = createPosXSlice(shaftBracket, dh)
            CSG connectionShaftBracketMount = createPosXConnectionMount(shaftBracketSlice)
            shaftBracket = moveDHValues(shaftBracket, dh)
            connectionShaftBracketMount = moveDHValues(connectionShaftBracketMount, dh)

            def endEffector = new Cube(10.0, 10.0, 80.0).toCSG()
            endEffector.setColor(Color.DARKOLIVEGREEN)

            CSG endEffectorSlice = createNegXSlice(endEffector)
            CSG connectionEndEffectorMount = createNegXConnectionMount(endEffectorSlice)

            def connection = connectionShaftBracketMount.hull(connectionEndEffectorMount)

            CSG motorKeepawayCylinder = createMotorKeepawayCylinder(motorCad, dh)
            connection = connection.difference(motorKeepawayCylinder)
            connection.setColor(Color.MEDIUMPURPLE)
            connectionShaftBracketMount.setColor(Color.MEDIUMPURPLE)
            connectionEndEffectorMount.setColor(Color.MEDIUMPURPLE)

            def linkCSGs = [endEffector, shaftBracket, connection]
            linkCSGs.each {
                it.setManipulator(dh.getListener())
            }
            allCad.addAll(linkCSGs)
        }

        double totalMassKg = 0.0
        TransformNR centerOfMassFromCentroid = new TransformNR()
        def intermediateCoMs = new ArrayList<TransformNR>()

        for (TransformNR vitaminLocation : vitaminLocations.keySet()) {
            def vitaminType = vitaminLocations.get(vitaminLocation)[0]
            def vitaminSize = vitaminLocations.get(vitaminLocation)[1]
            def vitaminData = Vitamins.getConfiguration(vitaminType, vitaminSize)
            def vitaminCad = Vitamins.get(vitaminType, vitaminSize)

            println("Adding vitamin to CoM: " + vitaminData)

            def massKg = vitaminData["massKg"] as double
            def comCentroid = vitaminLocation.times(
                    new TransformNR(
                            vitaminData["massCentroidX"] as double,
                            vitaminData["massCentroidY"] as double,
                            vitaminData["massCentroidZ"] as double,
                            new RotationNR()
                    )
            )

            def cad = vitaminCad.transformed(TransformFactory.nrToCSG(vitaminLocation))
            cad.setManipulator(manipulator)
            allCad.add(cad)

            totalMassKg += massKg
            intermediateCoMs.add(comCentroid)
            centerOfMassFromCentroid.x = (centerOfMassFromCentroid.x + comCentroid.x) * massKg
            centerOfMassFromCentroid.y = (centerOfMassFromCentroid.y + comCentroid.y) * massKg
            centerOfMassFromCentroid.z = (centerOfMassFromCentroid.z + comCentroid.z) * massKg
        }

        centerOfMassFromCentroid.x = centerOfMassFromCentroid.x / totalMassKg
        centerOfMassFromCentroid.y = centerOfMassFromCentroid.y / totalMassKg
        centerOfMassFromCentroid.z = centerOfMassFromCentroid.z / totalMassKg

        conf.setMassKg(totalMassKg)
        conf.setCenterOfMassFromCentroid(centerOfMassFromCentroid)
        println("Computed totalMassKg=" + totalMassKg)
        println("Computed centerOfMassFromCentroid=" + centerOfMassFromCentroid)

        return allCad
    }

    private static List sliceLinkConnection(DHLink dh, CSG link, CSG motorCad, CSG nextMotorCad) {
        // In this function, "top" means the end of this link, "bottom" means the start of this
        // link, "left" means positive z in the frame of the motor attached to this link, and
        // "right" means negative z in the frame of the motor attached to this link.
        def topBottomSliceCube = new Cube(170).toCSG()
                .toXMin()
                .transformed(
                        TransformFactory.nrToCSG(
                                new TransformNR(dh.DhStep(0))
                                        .inverse()
                                        .scale(0.5)
                        )
                )

        def topHalf = link.intersect(topBottomSliceCube)
        def bottomHalf = link.difference(topBottomSliceCube)

        def bottomLeftSliceCube = new Cube(170).toCSG()
                .toZMin()
                .movez(motorCad.minZ / 2)
                .transformed(
                        TransformFactory.nrToCSG(new TransformNR(dh.DhStep(0)).inverse())
                )

        def topLeftSliceCube = new Cube(170).toCSG()
                .toZMin()
                .movez(nextMotorCad.minZ / 2)

        def bottomRightSliceCube = new Cube(170).toCSG()
                .toZMax()
                .movez(motorCad.minZ / 2)
                .transformed(
                        TransformFactory.nrToCSG(new TransformNR(dh.DhStep(0)).inverse())
                )

        def topRightSliceCube = new Cube(170).toCSG()
                .toZMax()
                .movez(nextMotorCad.minZ / 2)

        def leftHalf = CSG.unionAll([
                topLeftSliceCube.intersect(topHalf),
                bottomLeftSliceCube.intersect(bottomHalf)
        ])
        def rightHalf = CSG.unionAll([
                topRightSliceCube.intersect(topHalf),
                bottomRightSliceCube.intersect(bottomHalf)
        ])

        leftHalf.setColor(Color.CYAN)
        rightHalf.setColor(Color.MEDIUMPURPLE)

        return [leftHalf, rightHalf]
    }

    private static CSG createPosXSlice(CSG shaftBracket, DHLink dh) {
        def shaftBracketSlice = new Cube(
                0.1,
                shaftBracket.totalY,
                shaftBracket.totalZ
        ).toCSG()
        shaftBracketSlice = shaftBracketSlice
                .toXMax()
                .movex(shaftBracket.maxX)
                .movey(shaftBracket.centerY)
                .movez(shaftBracket.centerZ)
        shaftBracketSlice = shaftBracketSlice.intersect(shaftBracket)
        shaftBracketSlice.setColor(Color.BLACK)
        shaftBracketSlice
    }

    private static CSG createNegXSlice(CSG motorBracket) {
        def motorBracketSlice = new Cube(
                0.1,
                motorBracket.totalY,
                motorBracket.totalZ
        ).toCSG()
        motorBracketSlice = motorBracketSlice
                .toXMin()
                .movex(motorBracket.minX)
                .movey(motorBracket.centerY)
                .movez(motorBracket.centerZ)
        motorBracketSlice = motorBracketSlice.intersect(motorBracket)
        motorBracketSlice.setColor(Color.BLACK)
        motorBracketSlice
    }

    private static CSG createPosXConnectionMount(CSG posXSlice) {
        CSG connectionShaftBracketMount = new Cube(5.0, posXSlice.totalY, posXSlice.totalZ).toCSG()
        connectionShaftBracketMount = connectionShaftBracketMount
                .toXMin()
                .movex(posXSlice.maxX)
                .movey(posXSlice.centerY)
                .movez(posXSlice.centerZ)
        return connectionShaftBracketMount
    }

    private static CSG createNegXConnectionMount(CSG negXSlice) {
        CSG connectionEndEffectorMount = new Cube(5.0, negXSlice.totalY, negXSlice.totalZ).toCSG()
        connectionEndEffectorMount = connectionEndEffectorMount
                .toXMax()
                .movex(negXSlice.minX)
                .movey(negXSlice.centerY)
                .movez(negXSlice.centerZ)
        return connectionEndEffectorMount
    }

    private static CSG createMotorKeepawayCylinder(CSG motorCad, DHLink dh) {
        // Create a cylinder that encases the motor body and difference it from the
        // linkBracket. The z axis of the cylinder needs to be the same as the z axis of
        // the motor (because that's the rotation axis of the link). The radius of the
        // cylinder needs to be big enough to encompass the entire motor if it was revolved
        // 360 degrees around its z axis.
        double motorMaxX = Math.max(Math.abs(motorCad.maxX), Math.abs(motorCad.minX))
        double motorMaxY = Math.max(Math.abs(motorCad.maxY), Math.abs(motorCad.minY))
        double motorCylinderRadius = Math.sqrt(
                Math.pow(motorMaxX, 2) + Math.pow(motorMaxY, 2)
        ) + 5.0 // Add a bit for extra keepaway
        CSG motorKeepawayCylinder = new Cylinder(
                motorCylinderRadius, motorCad.totalZ
        ).toCSG()
        motorKeepawayCylinder = motorKeepawayCylinder.movez(
                -motorKeepawayCylinder.maxZ + motorCad.maxZ
        )
        motorKeepawayCylinder = moveDHValues(motorKeepawayCylinder, dh)
        motorKeepawayCylinder
    }
}

return new MyCadGen()
