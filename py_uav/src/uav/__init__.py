"""Nara TEKNOFEST sabit-kanat IHA otonomi sistemi (v10, Python).

Tasarim felsefesi: "Python plans, ArduPlane flies."

Companion bilgisayar (bu kod) yalnizca UST DUZEY kararlar verir:
  - verilen waypoint listesini HSS/fence-guvenli hale getirir,
  - sabit-kanat donus yaricapina gore guvenli koridor marji birakir,
  - GUIDED modda DO_REPOSITION ile waypoint'leri sirayla gonderir.

Gercek ucus (L1/NPFG + TECS), donus, loiter, stall/fence korumasi
ArduPlane'in icinde calisir ve her zaman aktiftir. Companion ASLA
hiz (velocity) komutu gondermez -- ArduPlane sabit-kanatta bunu yok sayar.
"""

__version__ = "10.0.0"
