from math import sin, cos, pi
import copy


def z_pliku(nazwa_pliku):
    f = open(nazwa_pliku, "r")
    linijki = []
    for line in f:
        linijki.append(line.split())
    f.close()
    return linijki


def zmiana_na_pi(wartosc):
    return (wartosc*pi)/3.14


def z_dh_do_urdf(dane):
    # Trzy pierwsze komorki to wartosci xyz a trzy nastepne to rpy.
    # Ostatnia komorka to typ laczenia:
    # 0 - fixed
    # 1 - revolute
    # 2 - prismatic
    urdf = [None] * 11
    a = float(dane[1])
    d = float(dane[2])
    urdf[0] = str(round(a, 5))
    urdf[1] = str(round(d * sin(zmiana_na_pi(float(dane[0]))) * (-1), 5))
    urdf[2] = str(round(d * cos(zmiana_na_pi(float(dane[0]))), 5))
    urdf[3] = str(round(zmiana_na_pi(float(dane[0])), 5))
    urdf[4] = "0"
    urdf[5] = str(round(zmiana_na_pi(float(dane[3])), 5))
    laczenie = {
        0: "fixed",
        1: "revolute",
        2: "prismatic"
    }
    urdf[6] = laczenie.get(int(dane[4]))
    urdf[7] = dane[5]
    urdf[8] = str(round(a/2, 5))
    urdf[9] = str(round((-1) * d / 2, 5))
    urdf[10] = str(round((-1) * d * cos(zmiana_na_pi(float(dane[0]))), 5))
    return urdf


def stworz_plik():
    dane_z_pliku = copy.deepcopy(z_pliku("dh.txt"))
    poprzednia_nazwa = 'baza'
    f = open("manipulator.urdf.xml", "w")
    f.write('<robot name="robot">\n\n')
    f.write('    <link name="baza">\n')
    f.write('            <visual>\n')
    f.write('                <geometry>\n')
    f.write('                    <cylinder length="1" radius="0.1"/>\n')
    f.write('                </geometry>\n')
    f.write('                <origin rpy="0 0 0" xyz="0 0 0.5"/>\n')
    f.write('                <material name="gray">\n')
    f.write('                    <color rgba="0.5 0.5 0.5 1"/>\n')
    f.write('                </material>\n')
    f.write('            </visual>\n')
    f.write('    </link>\n\n')
    for wartosci in range(1, len(dane_z_pliku)):
        dane_do_urdf = z_dh_do_urdf(dane_z_pliku[wartosci])
        z = '0.0'
        x = '0.0'
        x_size = '0.0'
        if wartosci == len(dane_z_pliku)-1:
            x_size = '0.05'
            x = '0.025'
        else:
            dane_dodatkowe = copy.deepcopy(z_dh_do_urdf(dane_z_pliku[wartosci+1]))
            x_size = copy.copy(dane_dodatkowe[0])
            x = copy.copy(dane_dodatkowe[8])
        if dane_do_urdf[6] != 'prismatic':
            dane_do_urdf[9] = '0.025'
            z = '0.05'
        else:
            z = dane_do_urdf[10]

        f.write('    <link name="'+dane_do_urdf[7]+'">\n')
        f.write('        <visual>\n')
        f.write('            <geometry>\n')
        f.write('                <box size="'+x_size+' 0.05 '+z+'"/>\n')
        f.write('            </geometry>\n')
        f.write('            <origin xyz="'+x+' 0 '+dane_do_urdf[9]+'" rpy="0 0 0"/>\n')
        f.write('            <material name="gray">\n')
        f.write('                <color rgba="0.5 0.5 0.5 1"/>\n')
        f.write('            </material>\n')
        f.write('        </visual>\n')
        f.write('    </link>\n\n')

        f.write('    <joint name="joint_'+str(wartosci)+'" type="'+dane_do_urdf[6]+'">\n')
        f.write('        <parent link="'+poprzednia_nazwa+'"/>\n')
        f.write('        <child link="'+dane_do_urdf[7]+'"/>\n')
        if dane_do_urdf[6] == "fixed":
            pass
        else:
            f.write('        <axis xyz="0 0 1"/>\n')
            if dane_do_urdf[6] == "prismatic":
                f.write('        <limit upper="5.0" lower="-5.0" effort="10" velocity="10" />\n')
            elif dane_do_urdf[6] == "revolute":
                f.write('        <limit upper="3.14" lower="-3.14" effort="10" velocity="10" />\n')
        f.write('        <origin xyz="'+dane_do_urdf[0]+' '+dane_do_urdf[1]+' '+dane_do_urdf[2]+'" rpy="'+dane_do_urdf[3]+' '+dane_do_urdf[4]+' '+dane_do_urdf[5]+'"/>\n')
        f.write('    </joint>\n')
        f.write('\n')
        poprzednia_nazwa = dane_do_urdf[7]
    f.write('</robot>')
    f.close()


stworz_plik()
