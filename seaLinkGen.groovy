import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
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
    private double rearShaftBracketWidth = rearMotorBracketWidth
    private double rearShaftBracketThickness = rearMotorBracketThickness
    private CSG passiveHingeHeatedInsert = Vitamins.get("heatedThreadedInsert", "M5")
    private CSG passiveHingeShoulderBoltKeepaway = Vitamins.get("capScrew", "M5")
    private double linkClamshellBoltKeepawayRadius = 2.5
    private double linkClamshellBoltInset = 2
    private double bridgeThickness = linkClamshellBoltKeepawayRadius * 2 + linkClamshellBoltInset * 2

    MyCadGen() {
        CSG.setUseStackTraces(false)
    }

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

    /**
     * @param motorCSG The motor that is going to be mounted to the bracket (which is the motor on
     * the end of the link, not the motor powering the link).
     * @param link This link.
     * @param heatedInsert Used to make a space to insert a heated insert to form one half of the
     * passive hinge.
     * @return The motor bracket.
     */
    def makeMotorBracket(CSG motorCSG, CSG heatedInsert) {
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
        bridge = bridge.toXMax().movex(frontAndRearBrackets.minX)
        bridge = bridge.movey(motorCSG.centerY)

        def bracket = CSG.unionAll([frontAndRearBrackets, bridge])
        // Make space for the heated insert for the hinge
                .difference(heatedInsert.movez(rearMotorMountBracket.minZ))

        def linkClamshellBoltKeepaway = createLinkClamshellBoltKeepaway(bridge, bracket)
        bracket = bracket.difference(linkClamshellBoltKeepaway)

        bracket.setColor(Color.BURLYWOOD)
        return [bracket, linkClamshellBoltKeepaway]
    }

    /**
     * @param motorCSG The motor that is powering this link (not the motor bolted in to this link,
     * which is the next motor).
     * @param shaftCSG The shaft attached to the motor.
     * @param shaftCollar The shaft collar that will go on the shaft to connect this shaft bracket
     * to the shaft.
     * @param link This link.
     * @param shoulderBolt Used to create a space for the shoulder part of a shoulder bolt
     * to pass through to form half of the passive hinge. The threaded end of the shoulder bolt
     * screws in to a heated insert on the motor bracket for the motor bolted in to this link (i.e.,
     * the next motor).
     * @return The shaft bracket.
     */
    def makeShaftBracket(CSG motorCSG, CSG shaftCSG, CSG shaftCollar, CSG shoulderBolt) {
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
//        def g = TransformFactory.nrToCSG(new TransformNR(frontShaftMountBracket.maxX, 0, 0, new RotationNR(0, 0, 90)))
//        frontShaftMountBracket = frontShaftMountBracket.union(linkRFront,
//                Fillet.outerFillet(
//                        Slice.slice(
//                                frontShaftMountBracket,
//                                g,
//                                0
//                        ),
//                        5
//                ).transformed(g)
//        )

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

        // Make a space for the shoulder bolt for the hinge
        rearShaftMountBracket = rearShaftMountBracket.difference(
                shoulderBolt
                // Flip it upside down because it attaches to the rear bracket
                        .rotx(180)
                // Move the origin of the bolt to the outer edge of the bracket so the head is fully outside the bracket
                        .movez(-rearShaftBracketThickness)
        )

        // Line up centered with the motor body and at the end of the rear motor bracket
        rearShaftMountBracket = rearShaftMountBracket.movez(motorCSG.minZ)
        rearShaftMountBracket = rearShaftMountBracket.movez(-rearMotorBracketThickness)

        def frontAndRearBrackets = CSG.unionAll([frontShaftMountBracket, rearShaftMountBracket])
        def bridge = new Cube(bridgeThickness, motorCSG.totalY, frontAndRearBrackets.totalZ).toCSG()
        bridge = bridge.toZMax().movez(frontAndRearBrackets.maxZ)
        bridge = bridge.toXMax().movex(frontAndRearBrackets.maxX)

        def bracket = CSG.unionAll([frontAndRearBrackets, bridge])
        bracket = bracket.difference([
                shaftCollar.movez(motorCSG.maxZ),
                shaftCSG.movez(motorCSG.maxZ)
        ])

        def linkClamshellBoltKeepaway = createLinkClamshellBoltKeepaway(bridge, bracket)
        bracket = bracket.difference(linkClamshellBoltKeepaway)

        bracket.setColor(Color.CYAN)
        return [bracket, linkClamshellBoltKeepaway]
    }

    private static CSG getEncompassingCylinder(CSG csg) {
        double diam = Math.sqrt(Math.pow(csg.totalX, 2) + Math.pow(csg.totalY, 2))
        CSG out = new Cylinder(diam / 2, csg.totalZ).toCSG()
        out = out.move(csg.center)
        out = out.movez(-(out.maxZ - csg.maxZ))
        return out
    }

    @Override
    ArrayList<CSG> generateBody(MobileBase mobileBase) {
        Map<TransformNR, List<String>> vitaminLocations = new HashMap<TransformNR, List<String>>()
        List<CSG> allCad = new ArrayList<CSG>()

        for (DHParameterKinematics d : mobileBase.getAllDHChains()) {
            LinkConfiguration conf = d.getLinkConfiguration(0)
            LinkConfiguration nextConf = d.getLinkConfiguration(1)
            DHLink dh = d.getChain().getLinks()[0]
            DHLink nextDh = d.getChain().getLinks()[1]
            CSG motor = Vitamins.get("hobbyServo", "standardMicro")
            // Vitamins.get("hobbyServo", "standardMicro") Vitamins.get("stepperMotor", "GenericNEMA14") Vitamins.get("roundMotor", "WPI-gb37y3530-50en")
            CSG motorShaft = Vitamins.get("hobbyServoHorn", "standardMicro1")
            // Vitamins.get("hobbyServoHorn", "standardMicro1") Vitamins.get("dShaft", "5mm") Vitamins.get("dShaft", "WPI-gb37y3530-50en")
            double pitch = 3.0
            double thickness = 10.0
            double gearDiameter = 30.0
            double gearTeeth = (gearDiameter * Math.PI) / pitch
//            List<Object> gearGenResult = ScriptingEngine.gitScriptRun(
//                    "https://github.com/madhephaestus/GearGenerator.git",
//                    "bevelGear.groovy",
//                    [gearTeeth, gearTeeth, thickness, pitch] as ArrayList<Object>
//            ) as List<Object>
//            CSG gearL = gearGenResult[0] as CSG
//            CSG gearR = gearGenResult[1] as CSG
//            double gearSeparationDistance = gearGenResult[2] as double
            CSG gearL = new Cylinder(gearDiameter, thickness).toCSG()
            CSG gearR = new Cylinder(gearDiameter, thickness).toCSG()
            double gearSeparationDistance = -(gearR.maxY - gearL.minY) + 1

            CSG base = new Cube(
                    gearL.totalX,
                    gearL.totalY + gearR.totalY,
                    motor.totalZ
            ).toCSG()
            base = base.movey(-base.maxY / 2)

            // Put the thrust bearing in
            CSG thrustBearing = Vitamins.get("ballBearing", "Thrust_1andAHalfinch")
            thrustBearing = thrustBearing.toZMin().movez(base.maxZ - thrustBearing.totalZ + 0.5)
            base = base.difference(thrustBearing.hull())

            // Cut a path for the bolt to be inserted
            double baseBoltKeepawayRadius = 5
            CSG boltKeepaway = new Cylinder(baseBoltKeepawayRadius, 10000).toCSG()
            boltKeepaway = boltKeepaway.toZMax().movez(thrustBearing.minZ - 5)
            base = base.difference(boltKeepaway)

            // Cut a path for the bolt threads
            CSG bolt = Vitamins.get("capScrew", "M5x25")
            bolt = bolt.rotx(180).movez(boltKeepaway.maxZ)
            base = base.difference(bolt)

            // Add gears
            gearL = gearL.toZMin().movez(thrustBearing.maxZ)
            gearR = gearR.toZMin().movez(thrustBearing.maxZ).movey(gearSeparationDistance)

            // Cut a path for the bolt threads through gearL
            gearL = gearL.difference(bolt)

            // Add a bearing flush with the gear for the nut to sit on
            CSG nutBearing = Vitamins.get("ballBearing", "695zz")
            nutBearing = nutBearing.toZMin().movez(gearL.maxZ - nutBearing.totalZ)
            gearL = gearL.difference(nutBearing.hull())

            // Add a nut for the bolt
            CSG nut = Vitamins.get("lockNut", "M5")
            nut = nut.movez(gearL.maxZ)

            // Add the motor
            motor = motor.toZMax().movey(gearR.centerY).movez(base.maxZ)
            base = base.difference(motor)

            // Add the motor's shaft
            motorShaft = motorShaft.movey(gearR.centerY).movez(base.maxZ)
            gearR = gearR.difference(motorShaft)
            CSG shaftKeepaway = getEncompassingCylinder(motorShaft)
            shaftKeepaway = shaftKeepaway.movez(-(shaftKeepaway.maxZ - motor.maxZ))
            base = base.difference(shaftKeepaway)

            // Add the motor bracket
            def nextMotorType = nextConf.getElectroMechanicalType()
            def nextMotorSize = nextConf.getElectroMechanicalSize()
            def nextMotorCad = Vitamins.get(nextMotorType, nextMotorSize)
            def (CSG motorBracket, CSG motorBracketLinkClamshellBoltKeepaway) = makeMotorBracket(nextMotorCad, passiveHingeHeatedInsert)
//            motorBracket.setManipulator(dh.getListener())
            CSG motorBracketSlice = createNegXSlice(motorBracket)
            CSG connectionMotorBracketMount = createNegXConnectionMount(motorBracketSlice)
            connectionMotorBracketMount = reverseDHValues(connectionMotorBracketMount, dh)
            dh.getListener()

            // Add the link
            CSG link = new Cylinder(gearDiameter - 5, 30).toCSG()
            link = link.toZMin().movez(gearL.maxZ)
            // Keepaway for the nut and bolt, plus a channel in from the side for a wrench
            CSG nutAndBoltKeepaway = getEncompassingCylinder(bolt.union(nut))
            // Scale up to fit a wrench
            nutAndBoltKeepaway = nutAndBoltKeepaway.scalez(1.05).scalex(1.5).scaley(1.5)
            // Move to the edge of the link and hull to create a channel for the wrench to pass through
            nutAndBoltKeepaway = nutAndBoltKeepaway.union(nutAndBoltKeepaway.movex(link.minX)).hull()
            link = link.difference(nutAndBoltKeepaway)

            CSG connection = link.hull(connectionMotorBracketMount)

            CSG assembly = CSG.unionAll([base, thrustBearing, bolt, nut, gearL.union(connection), gearR, nutBearing, motor, motorShaft])
            assembly = assembly.toZMin()

            // Move it the root of the limb
            assembly = assembly.transformed(TransformFactory.nrToCSG(d.getRobotToFiducialTransform()))

            allCad.add(assembly)
        }

//        mobileBase.setMassKg(totalMass)
//        mobileBase.setCenterOfMassFromCentroid(centerOfMassFromCentroid)

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
                def (CSG motorBracket, CSG motorBracketLinkClamshellBoltKeepaway) = makeMotorBracket(nextMotorCad, passiveHingeHeatedInsert)

                CSG shaftCollar
                if (motorType == "hobbyServo") {
                    // The shaft for a servo is the horn, so just difference that
                    shaftCollar = shaftCad
                } else {
                    // Otherwise we need to bolt something onto the shaft to mesh with the link
                    // bracket, so use a shaft collar
                    shaftCollar = Vitamins.get("brushlessBoltOnShaft", "sunnysky_x2204")
                }
                def (CSG shaftBracket, CSG shaftBracketLinkClamshellBoltKeepaway) = makeShaftBracket(motorCad, shaftCad, shaftCollar, passiveHingeShoulderBoltKeepaway)

                CSG motorBracketSlice = createNegXSlice(motorBracket)
                CSG connectionMotorBracketMount = createNegXConnectionMount(motorBracketSlice)

                CSG shaftBracketSlice = createPosXSlice(shaftBracket, dh)
                CSG connectionShaftBracketMount = createPosXConnectionMount(shaftBracketSlice)
                shaftBracket = moveDHValues(shaftBracket, dh)
                connectionShaftBracketMount = moveDHValues(connectionShaftBracketMount, dh)

                def connection = connectionMotorBracketMount.hull(connectionShaftBracketMount)

                CSG motorKeepawayCylinder = createMotorKeepawayCylinder(motorCad, dh)
                connection = connection.difference([
                        motorKeepawayCylinder,
                        motorBracket.scalez(2).hull(),
                        shaftBracket.hull(),
                        moveDHValues(reverseDHValues(shaftBracket.scalez(2).hull(), dh), dh), // TODO: This needs to be scalex for alpha=90
                        motorBracketLinkClamshellBoltKeepaway,
                        moveDHValues(shaftBracketLinkClamshellBoltKeepaway, dh)
                ])

                def link = CSG.unionAll([motorBracket, shaftBracket, connection])
                def (CSG posZHalf, CSG negZHalf) = sliceLink(dh, link, motorCad, nextMotorCad)
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
            def (CSG shaftBracket, CSG shaftBracketLinkClamshellBoltKeepaway) = makeShaftBracket(motorCad, shaftCad, shaftCollar, passiveHingeShoulderBoltKeepaway)

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
            connection = connection.difference([
                    motorKeepawayCylinder,
                    moveDHValues(shaftBracketLinkClamshellBoltKeepaway, dh),
                    moveDHValues(reverseDHValues(shaftBracket.scalez(2).hull(), dh), dh)
            ])

            def link = CSG.unionAll([endEffector, shaftBracket, connection])
            def (CSG posZHalf, CSG negZHalf) = sliceLink(dh, link, motorCad, endEffector)
            def linkCSGs = [posZHalf, negZHalf]
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

    private List sliceLink(DHLink dh, CSG link, CSG motorCad, CSG nextMotorCad) {
        // In this function, "top" means the end of this link, "bottom" means the start of this
        // link, "left" means positive z in the frame of the motor attached to this link, and
        // "right" means negative z in the frame of the motor attached to this link.
        def dhTransform = TransformFactory.nrToCSG(new TransformNR(dh.DhStep(0)).inverse())
        def topBottomSliceCube = new Cube(10000).toCSG()
                .toXMin()
        // Apply dhTransform but without theta or alpha
                .movez(-dh.d / 2)
                .movex(-dh.r / 2)

        def topHalf = link.intersect(topBottomSliceCube)
        def bottomHalf = link.difference(topBottomSliceCube)

        def bottomLeftSliceCube = new Cube(10000).toCSG()
                .toZMin()
                .movez(motorCad.minZ / 2)
                .transformed(dhTransform)

        def topLeftSliceCube = new Cube(10000).toCSG()
                .toZMin()
                .movez(nextMotorCad.minZ / 2)

        def bottomRightSliceCube = new Cube(10000).toCSG()
                .toZMax()
                .movez(motorCad.minZ / 2)
                .transformed(dhTransform)

        def topRightSliceCube = new Cube(10000).toCSG()
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

    private CSG createPosXConnectionMount(CSG posXSlice) {
        CSG connectionShaftBracketMount = new Cube(
                1.0,
                posXSlice.totalY,
                posXSlice.totalZ
        ).toCSG()

        connectionShaftBracketMount = connectionShaftBracketMount
                .toXMin()
                .movex(posXSlice.maxX)
                .movey(posXSlice.centerY)
                .movez(posXSlice.centerZ)
        return connectionShaftBracketMount
    }

    private CSG createNegXConnectionMount(CSG negXSlice) {
        CSG connectionEndEffectorMount = new Cube(
                1.0,
                negXSlice.totalY,
                negXSlice.totalZ
        ).toCSG()

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

    private CSG createLinkClamshellBoltKeepaway(CSG bridge, CSG bracket) {
        def linkClamshellBoltKeepaway = new Cylinder(linkClamshellBoltKeepawayRadius, 10000).toCSG()
                .toZMax().movez(bracket.maxZ)
                .movex(bridge.minX + linkClamshellBoltKeepawayRadius + linkClamshellBoltInset)
        return CSG.unionAll([
                linkClamshellBoltKeepaway.toYMax().movey(bracket.maxY - linkClamshellBoltInset),
                linkClamshellBoltKeepaway.toYMin().movey(bracket.minY + linkClamshellBoltInset)
        ])
    }
}

return new MyCadGen()
