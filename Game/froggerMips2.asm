.text
main: 

startGame:
jal initFrogger
jal resetEnemy1
jal resetEnemy2
jal resetEnemy3
jal initializeLog
jal moveLake1
jal initializeEndGame
jal initializeLargeValue
jal initializeLeftScreenBound
jal initializeFurtherLeftScreenBound
jal initializeLives
jal initializeCollisionVar
jal initializeEndGameVar
jal initializeInsideLogVariable
jal initNeg300
jal initWinGame

goToWait: jal wait
decrementValueEnemy1: addi $r3, $r3, -1
decrementValueEnemy2: addi $r4, $r4, -2
jal checkCollision
returnToMain: jal checkEndGame
returnToMain2: bne $r3, $r20, checkEnemy2
jal resetEnemy1
checkEnemy2: bne $r4, $r20, goToWait
jal resetEnemy2
j goToWait

wait: addi $r19, $r0, 0
waitAdd: addi $r19, $r19, 1
bne $r19, $r18, waitAdd
jr $r31

checkCollision:
bne $r28, $r22, returnToMain
addi $r1, $r0, 0
addi $r2, $r0, 0
addi $r23, $r23, -1
bne $r23, $r0, returnFromCheckCollision
j endOfGamePause
returnFromCheckCollision: jr $r31

checkEndGame:
bne $r28, $r24, returnToMain2
addi $r1, $r0, 0
addi $r2, $r0, 0
j beginLevelTwo
jr $r31


initWinGame:
addi $r10, $r0, 0
jr $r31

resetEnemy1:
addi $r3, $r0, 1200
jr $r31

resetEnemy2:
addi $r4, $r0, 800
jr $r31

resetEnemy3:
addi $r5, $r0, 485
jr $r31

initializeLargeValue:
addi $r18, $r0, 25000
jr $r31

initializeLeftScreenBound:
addi $r20, $r0, 0
jr $r31

initializeFurtherLeftScreenBound:
addi $r26, $r0, 230
jr $r31

initializeLives:
addi $r23, $r0, 5
jr $r31

initializeCollisionVar:
addi $r22, $r0, 1
jr $r31

initializeEndGameVar:
addi $r24, $r0, 2
jr $r31

initNeg300:
addi $r27, $r0, -300
jr $r31

initializeInsideLogVariable:
addi $r25, $r0, 4
jr $r31

initializeEndGame:
addi $r11, $r0, 440
jr $r31

initFrogger:
addi $r1, $r0, 0
addi $r2, $r0, 0
jr $r31

moveLake1:
addi $r7, $r0, 2500
jr $r31

initLake1:
addi $r7, $r0, 435
jr $r31

endOfGamePause:
addi $r10, $r0, 2
halt
j startGame

moveEnemy1:
addi $r3, $r0, 1500
jr $r31

moveEnemy2:
addi $r4, $r0, 1500
jr $r31

moveEnemy3:
addi $r5, $r0, 1500
jr $r31

initializeLog:
addi $r6, $r0, 1200
jr $r31

checkCollision2:
bne $r28, $r22, returnToMainLevel2
addi $r1, $r0, 0
addi $r2, $r0, 0
addi $r23, $r23, -1
bne $r23, $r0, returnFromCheckCollisionLevel2
j endOfGamePause
returnFromCheckCollisionLevel2: jr $r31

checkEndGame2:
bne $r28, $r24, returnToMain2Level2
addi $r1, $r0, 0
addi $r2, $r0, 0
addi $r10, $r0, 1
halt
jr $r31

beginLevelTwoMinusOne:
addi $r23, $r23, -1

beginLevelTwo:
jal moveEnemy1
jal moveEnemy2
jal moveEnemy3
jal initFrogger
jal initLake1


goToWaitLevel2: jal wait
decrementValueLog: addi $r6, $r6, -1
bne $r28, $r25, skipMoveWithLog
blt $r1, $r27, beginLevelTwoMinusOne
addi $r1, $r1, -1
skipMoveWithLog: jal checkCollision2
returnToMainLevel2: jal checkEndGame2
returnToMain2Level2: bne $r6, $r20, goToWaitLevel2
jal initializeLog
j goToWaitLevel2


halt




.data