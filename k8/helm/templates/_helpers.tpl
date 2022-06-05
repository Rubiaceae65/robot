

{{- define "myNamedTemplate" }}
    - name: {{ .Values.myParam1 }}
      value: {{ .Values.myValue1 | quote }}
    - name: {{ .Values.myParam2 }}
      value: {{ .Values.myValue2 | quote }}
{{- end }}

{{- $myValues := dict "myParam1" "myValue1" "myParam2" "myValue2" -}}
{{- $myParameters := dict "Values" $myValues -}}
{{- include "myNamedTemplate" $myParameters }}


{{/*
Env section
*/}}
{{- define "robot.container" -}}
- name: gzweb
  image: 'lala432/ros-dev:latest'
  imagePullPolicy: Always
  resources:
    limits:
      memory: "4Gi"
    requests:
      memory: "2Gi"
  command: 
    - /scripts/ros-run.sh
  env:
    - name: ROS_WS
      value: /home/user/ros-workspace
    - name: ROS_MASTER_URI
      value: http://robot-ros-master:11311/ 
    - name: LAUNCH
      value: gzweb
  volumeMounts:
    - name: scripts
      mountPath: /scripts
    - name: launch
      mountPath: /launch

{{- end }}



{{/*
Expand the name of the chart.
*/}}
{{- define "robot.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Create a default fully qualified app name.
We truncate at 63 chars because some Kubernetes name fields are limited to this (by the DNS naming spec).
If release name contains chart name it will be used as a full name.
*/}}
{{- define "robot.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- if contains $name .Release.Name }}
{{- .Release.Name | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}
{{- end }}

{{/*
Create chart name and version as used by the chart label.
*/}}
{{- define "robot.chart" -}}
{{- printf "%s-%s" .Chart.Name .Chart.Version | replace "+" "_" | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Common labels
*/}}
{{- define "robot.labels" -}}
helm.sh/chart: {{ include "robot.chart" . }}
{{ include "robot.selectorLabels" . }}
{{- if .Chart.AppVersion }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
{{- end }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}

{{/*
Selector labels
*/}}
{{- define "robot.selectorLabels" -}}
app.kubernetes.io/name: {{ include "robot.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}

{{/*
Create the name of the service account to use
*/}}
{{- define "robot.serviceAccountName" -}}
{{- if .Values.serviceAccount.create }}
{{- default (include "robot.fullname" .) .Values.serviceAccount.name }}
{{- else }}
{{- default "default" .Values.serviceAccount.name }}
{{- end }}
{{- end }}
