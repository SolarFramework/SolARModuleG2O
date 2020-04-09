pipeline {
    agent {
        label "jenkins-slave-argo-qmake"
    }
    options {
        buildDiscarder(logRotator(numToKeepStr: '10'))
    }
    parameters {
        booleanParam(name: 'RELEASE',
            defaultValue: false,
            description: 'Create a Release')
    }
    triggers { pollSCM('* * * * *') }    
    stages {

        stage("Get Binaries") {
            when {
                expression { params.RELEASE == false }
            }            
            steps {
                script {
                    BRANCH = BRANCH_NAME.replace('/','_')
                }
                withCredentials([usernamePassword(credentialsId: 'bc6e150e-ede8-4e35-8af4-0f037edee8ac', passwordVariable: 'PASSWORD', usernameVariable: 'USERNAME')]) {
                    getBinaries(BRANCH,USERNAME,PASSWORD)
                }
            }
        }
     
        stage("Get Third Parties") {
             when {
                expression { params.RELEASE == true }
             }               
             steps {
                getThirdParties('1.0.0')
             }
        }

        stage("Get Third Parties dependencies") {
             when {
                expression { params.RELEASE == false }
             }               
             steps {
                installDependency('g2o','1.0.0')
             }
        }        

        stage("Build") {
             steps {
                 withEnv(['REMAKEN_RULES_ROOT=/home/jenkins/.remaken/rules/']) {
                     sh "qmake SolARModuleG2O.pro"
                     sh "make"
                     sh "make install"
                 }
             }
        }

        stage("Build Debug") {
            when {
                expression { params.RELEASE == true }
            }            
             steps {
                 withEnv(['REMAKEN_RULES_ROOT=/home/jenkins/.remaken/rules/']) {
                     sh "qmake SolARModuleG2O.pro CONFIG+=debug"
                     sh "make"
                     sh "make install"
                 }
             }
        }       

        // Trigger only if not a RELEASE
        stage("Share Binaries") {
            when {
                expression { params.RELEASE == false }
            }
            steps {
                withCredentials([usernamePassword(credentialsId: 'bc6e150e-ede8-4e35-8af4-0f037edee8ac', passwordVariable: 'PASSWORD', usernameVariable: 'USERNAME')]) {
                    shareBinaries(BRANCH,USERNAME,PASSWORD)
                }
            }
        }
        
        stage("Release") {
            when {
                expression { params.RELEASE == true }
            }
            steps {
                script {
                    version = sh (
                        script: "cat *.pro  | grep VERSION | head -1 | cut -d '=' -f 2",
                        returnStdout: true
                    )
                    version = version.replace("\n","")
                }                
                prepareRelease("SolARModuleG2O","linux-gcc","SolARModuleG2O")                
                withCredentials([string(credentialsId: 'github-token', variable: 'token')]) {
                    release("SolARModuleG2O","SolARModuleG2O/${version}/unix","",token);
                }
            }   
        }
}
}