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

        stage("Get Third Parties") {
             steps {
                 script {
                     BRANCH = BRANCH_NAME.replace('/','_')
                     version = getVersion()
                 }
                 withCredentials([string(credentialsId: 'artifactoryApiKey', variable: 'apiKey')]) {
                    installRemaken(params.RELEASE,BRANCH,apiKey)
                 }
             }
        }    
     
        stage("Build") {
             steps {
                buildModule("")
             }
        }

        stage("Build Debug") {
            when {
                expression { params.RELEASE == true }
            }            
             steps {
                buildModule("debug")
             }
        }       

        stage("Generate artifacts") {
            steps {
                script {
                    prepareRelease("SolARBuild","linux-gcc","SolARModuleG2O")                
                }
            }
        }  

        // Trigger only if not a RELEASE
        stage("Upload to Artifactory") {
            when {
                expression { params.RELEASE == false }
            }
            steps {
                withCredentials([usernamePassword(credentialsId: 'bc6e150e-ede8-4e35-8af4-0f037edee8ac', passwordVariable: 'PASSWORD', usernameVariable: 'USERNAME')]) {
                    uploadArchive(BRANCH,"SolARModuleG2O/${version}/linux","${WORKSPACE}/artifactory/x86_64_shared_release/SolARModuleG2O_${version}_x86_64_shared_release.zip",USERNAME,PASSWORD)
                }
            }
        }
        
		stage("Release") {
            when {
                expression { params.RELEASE == true }
            }
            steps {
                withCredentials([string(credentialsId: 'github-token', variable: 'token')]) {
                    release("SolARModuleG2O","SolARModuleG2O/${version}/linux","SolARModuleG2O Version ${version} for Linux","${WORKSPACE}/artifactory/x86_64_shared_release/SolARModuleG2O${version}_x86_64_shared_release.zip ${WORKSPACE}/artifactory/x86_64_shared_debug/SolARModuleG2O${version}_x86_64_shared_debug.zip",token);
                }
            }   
        }
}
}