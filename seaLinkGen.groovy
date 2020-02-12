import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.creature.CreatureLab
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color
import org.apache.commons.io.IOUtils
import com.neuronrobotics.bowlerstudio.vitamins.*
import java.nio.file.Paths
import eu.mihosoft.vrl.v3d.FileUtil
import com.neuronrobotics.bowlerstudio.vitamins.*
import javafx.scene.transform.Affine
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

class MyCadGen implements ICadGenerator {

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

    static CSG makeMotorBracket(CSG motorCSG, DHLink link) {
        CSG frontMotorMountBracket = new Cube(
                motorCSG.totalX,
                motorCSG.totalY,
                5.0
        ).toCSG()

        // Line up with the mounting face
        frontMotorMountBracket = frontMotorMountBracket.toZMin()

        // Line up the edges
        frontMotorMountBracket = frontMotorMountBracket.movex(motorCSG.centerX)
        frontMotorMountBracket = frontMotorMountBracket.movey(motorCSG.centerY)

        // Cut out mounting points
        frontMotorMountBracket = frontMotorMountBracket.difference(motorCSG)

        CSG rearMotorMountBracket = new Cube(
                motorCSG.totalX,
                motorCSG.totalY,
                5.0
        ).toCSG()

        // Line up with back face
        rearMotorMountBracket = rearMotorMountBracket.toZMax()
        rearMotorMountBracket = rearMotorMountBracket.movez(motorCSG.minZ)

        // Line up the edges
        rearMotorMountBracket = rearMotorMountBracket.movex(motorCSG.centerX)
        rearMotorMountBracket = rearMotorMountBracket.movey(motorCSG.centerY)

        def bracket = frontMotorMountBracket.union(rearMotorMountBracket)
        bracket.setManipulator(link.getListener())
        bracket.setColor(Color.BURLYWOOD)
        return bracket
    }

    static CSG makeShaftBracket(CSG motorCSG, CSG shaftCSG, CSG shaftCollar, DHLink link) {
        // Make a new size for brushlessBoltOnShaft and use it to mechanically link to the shaft
        // Accept the CSG for a brushlessBoltOnShaft as a parameter
        // But don't use it if the motor is a servo; in that case, use the servo horn instead
        double shaftBracketX = Math.max(shaftCSG.totalX + 10.0, motorCSG.totalX)
        double shaftBracketY = Math.max(shaftCSG.totalY + 10.0, motorCSG.totalY)

        CSG frontShaftMountBracket = new Cube(
                shaftBracketX,
                shaftBracketY,
                5.0
        ).toCSG()

        // Line up with the end of the motor and the start of the shaft
        frontShaftMountBracket = frontShaftMountBracket.toZMin()
        frontShaftMountBracket = frontShaftMountBracket.movez(motorCSG.maxZ)

        CSG rearShaftMountBracket = new Cube(
                shaftBracketX,
                shaftBracketY,
                5.0
        ).toCSG()

        // Line up with the opposite end of the motor
        rearShaftMountBracket = rearShaftMountBracket.toZMax()
        rearShaftMountBracket = rearShaftMountBracket.movez(motorCSG.minZ)

        def bracket = frontShaftMountBracket.union(rearShaftMountBracket)
        bracket = bracket.difference(shaftCollar.movez(motorCSG.maxZ))
        bracket.setManipulator(link.getListener())
        bracket.setColor(Color.CYAN)
        return bracket
    }

    @Override
    ArrayList<CSG> generateBody(MobileBase mobileBase) {
//        def vitaminLocations = new HashMap<TransformNR, List<String>>()
        CSG body = new Cube(30).toCSG()
//        TransformNR locationOfMotorMount = mobileBase.getRobotToFiducialTransform().copy()
//        vitaminLocations.put(
//                locationOfMotorMount,
//                ["ballBearing", "Thrust_1andAHalfinch"]
//        )
//        vitaminLocations.put(
//                locationOfMotorMount.copy(),
//                []
//        )
        body.setColor(Color.WHITE)
        body.setManipulator(mobileBase.getRootListener())
        return [body]
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
                DHLink prevDh = dhLinks.get(linkIndex - 1)

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
                def shaftBracket = makeShaftBracket(motorCad, shaftCad, shaftCollar, prevDh)

                def motorBracketSlice = new Cube(
                        0.1,
                        motorBracket.totalY,
                        motorBracket.totalZ
                ).toCSG()
                // Center it in the motor
                motorBracketSlice = motorBracketSlice.move(motorBracket.center)
                // Move it to the edge of the motor bracket
                motorBracketSlice = motorBracketSlice.movex(motorBracket.minX)
                // Get the slice of the bracket
                motorBracketSlice = motorBracketSlice.intersect(motorBracket)

                def shaftBracketSlice = new Cube(
                        0.1,
                        shaftBracket.totalY,
                        shaftBracket.totalZ
                ).toCSG()
                // Center it in the motor
                shaftBracketSlice = shaftBracketSlice.move(shaftBracket.center)
                // Move it to the edge of the motor bracket
                shaftBracketSlice = shaftBracketSlice.movex(shaftBracket.maxX)
                // Get the slice of the bracket
                shaftBracketSlice = shaftBracketSlice.intersect(shaftBracket)

                shaftBracketSlice = moveDHValues(shaftBracketSlice, dh)
                shaftBracket = moveDHValues(shaftBracket, dh)

                def connection = motorBracketSlice.hull(shaftBracketSlice)
                def linkBracket = CSG.unionAll([motorBracket, connection, shaftBracket])
                // Create a cylinder that encases the motor body and difference it from the linkBracket
                linkBracket.setManipulator(dh.getListener())
                allCad.add(linkBracket)
//                allCad.add(connection)
//                allCad.add(motorBracket)
//                allCad.add(shaftBracket)
//                allCad.add(motorBracketSlice)
//                allCad.add(shaftBracketSlice)
            }
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
}

return new MyCadGen()
