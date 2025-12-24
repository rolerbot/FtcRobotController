package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*


    * Acest OpMode ilustreaza cum sa folosesti DFRobot HuskyLens.
 *
 * HuskyLens este un senzor de viziune cu un model de detectare a obiectelor integrat. Poate
 * detecta un numar de obiecte predefinite si AprilTags din familia 36h11, poate
 * recunoaste culori si poate fi antrenat sa detecteze obiecte personalizate. Vezi acest website pentru
 * documentatie: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * Pentru instructiuni detaliate despre cum se foloseste HuskyLens in FTC, vezi acest tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 *
 * Acest exemplu ilustreaza cum sa detectezi AprilTags, dar poate fi folosit sa detecteze alte tipuri
 * de obiecte prin schimbarea algoritmului. Presupune ca HuskyLens este configurat cu
 * numele "huskylens".
 *
 * Foloseste Android Studio pentru a copia aceasta clasa si pentru a o lipi in folderul de cod al echipei cu un nume nou.
 * Sterge sau comenteaza linia @Disabled pentru a adauga acest OpMode la lista OpMode a Driver Station
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Husky", group="Linear Opmode")
public class SensorHuskyLens extends LinearOpMode {

    private final int PERIOADA_CITIRE = 1;

    private HuskyLens huskyLens;

    @Override
    public void runOpMode()
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * Acest exemplu limiteaza rata de citire doar pentru a permite utilizatorului timp sa observe
         * ce se intampla pe telemetria Driver Station. Aplicatiile tipice
         * nu ar limita rata.
         */
        Deadline limitaRata = new Deadline(PERIOADA_CITIRE, TimeUnit.SECONDS);

        /*
         * Expira imediat astfel incat prima data sa facem citirea.
         */
        limitaRata.expire();

        /*
         * Verificare de baza pentru a vedea daca dispozitivul este activ si comunica. Aceasta nu este
         * tehnic necesara aici deoarece clasa HuskyLens face asta in metoda
         * doInitialization() care este apelata cand dispozitivul este extras din
         * harta hardware. Totusi, uneori nu este clar de ce un dispozitiv raporteaza ca
         * a esuat la initializare. In cazul acestui dispozitiv, este pentru ca
         * apelul la knock() a esuat.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problema la comunicarea cu " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Apasa start pentru a continua");
        }

        /*
         * Dispozitivul foloseste conceptul de algoritm pentru a determina ce tipuri de
         * obiecte va cauta si/sau in ce mod este. Algoritmul poate fi
         * selectat folosind rotiata de pe dispozitiv, sau prin software dupa cum se arata in
         * apelul la selectAlgorithm().
         *
         * SDK-ul in sine nu presupune ca utilizatorul doreste un algoritm anume la
         * pornire, si deci nu seteaza un algoritm.
         *
         * Utilizatorii ar trebui, in general, sa aleaga explicit algoritmul pe care doresc sa il foloseasca
         * in OpMode prin apelarea selectAlgorithm() si prin transmiterea uneia dintre valorile
         * gasite in enumerarea HuskyLens.Algorithm.
         *
         * Alte optiuni de algoritm pentru FTC ar putea fi: OBJECT_RECOGNITION, COLOR_RECOGNITION sau OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();

        /*
             * Cautam AprilTags conform apelului la selectAlgorithm() de mai sus. O grila utila
         * pentru testare poate fi gasita la https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Retine din nou ca dispozitivul recunoaste doar familia 36h11 de tag-uri din fabrica.
         */
        while(opModeIsActive()) {
            if (!limitaRata.hasExpired()) {
                continue;
            }
            limitaRata.reset();

            /*
             * Toti algoritmii, cu exceptia LINE_TRACKING, returneaza o lista de Blocks unde un
             * Block reprezinta conturul unui obiect recunoscut impreuna cu numarul sau de ID.
             * Numerele de ID iti permit sa identifici ce a vazut dispozitivul. Vezi documentatia HuskyLens
             * mentionata in comentariul header de mai sus pentru mai multe informatii despre ID-uri si cum sa
             * le atribui la obiecte.
             *
             * Returneaza un array gol daca nu sunt vazute obiecte.
             */
            HuskyLens.Block[] blocuri = huskyLens.blocks();
            telemetry.addData("Numar blocuri", blocuri.length);
            for (int i = 0; i < blocuri.length; i++) {
                telemetry.addData("Bloc", blocuri[i].toString());
                /*
                 * Aici in interiorul buclei FOR, poti salva sau evalua informatii specifice pentru Bounding Box-ul curent recunoscut:
                 * - blocuri[i].width si blocuri[i].height   (dimensiunea cutiei, in pixeli)
                 * - blocuri[i].left si blocuri[i].top       (marginile cutiei)
                 * - blocuri[i].x si blocuri[i].y            (locatia centrului)
                 * - blocuri[i].id                           (ID-ul culorii)
                 *
                 * Aceste valori au tipul Java int (intreg).
                 */
            }

            telemetry.update();
        }
    }
}