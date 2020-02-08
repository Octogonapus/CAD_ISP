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
        CSG motorMountBracket = new Cube(
                motorCSG.totalX,
                motorCSG.totalY,
                5.0
        ).toCSG()

        // Line up with the mounting face
        motorMountBracket = motorMountBracket.toZMin()

        // Line up the edges
        motorMountBracket = motorMountBracket.movex(motorCSG.centerX)
        motorMountBracket = motorMountBracket.movey(motorCSG.centerY)

        // Cut out mounting points
        motorMountBracket = motorMountBracket.difference(motorCSG)

        motorMountBracket.setManipulator(link.getListener())
        motorMountBracket.setColor(Color.BURLYWOOD)
        return motorMountBracket
    }

    static CSG makeShaftBracket(CSG motorCSG, DHLink link) {
        CSG shaftMountBracket = new Cube(
                motorCSG.totalX,
                motorCSG.totalY,
                5.0
        ).toCSG()

        // Line up with the end of the motor
        shaftMountBracket = shaftMountBracket.toZMin()
        shaftMountBracket = shaftMountBracket.movez(motorCSG.maxZ)

        shaftMountBracket.setManipulator(link.getListener())
        shaftMountBracket.setColor(Color.CYAN)
        return shaftMountBracket
    }

    @Override
    ArrayList<CSG> generateBody(MobileBase mobileBase) {
        CSG body = new Cube(30).toCSG()
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

        if (linkIndex != d.getNumberOfLinks() - 1) {
            // If this is not the last link (the last link does not get a motor)
            // The motor for the first link is part of the base
            LinkConfiguration nextConf = d.getLinkConfiguration(linkIndex + 1)
            DHLink nextDh = dhLinks.get(linkIndex + 1)

            def motorType = nextConf.getElectroMechanicalType()
            def motorSize = nextConf.getElectroMechanicalSize()
            vitaminLocations.put(new TransformNR(), [motorType, motorSize])

            if (linkIndex != 0) {
                DHLink prevDh = dhLinks.get(linkIndex - 1)
                def motorCad = Vitamins.get(motorType, motorSize)
                def prevMotorCad = Vitamins.get(conf.getElectroMechanicalType(), conf.getElectroMechanicalSize())

                def motorBracket = makeMotorBracket(motorCad, dh)
                def shaftBracket = makeShaftBracket(prevMotorCad, prevDh)
                shaftBracket = moveDHValues(shaftBracket, dh)
                def linkBracket = motorBracket.hull(shaftBracket)
                linkBracket.setManipulator(dh.getListener())
                allCad.add(linkBracket)
//                allCad.add(motorBracket)
//                allCad.add(shaftBracket)
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
