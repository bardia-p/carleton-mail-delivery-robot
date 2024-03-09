const submitButton = $("#registerRobot-submit");
/**
 * The JavaScript AJAX call for when a new robot is created (i.e. the submit button is clicked).
 */

submitButton.click((e) => {
    e.preventDefault();
    const dataDictionary = {};
    dataDictionary["robotname"] = $("#robotname").val();
    const formData = JSON.stringify(dataDictionary);

    console.log(formData);
    console.log("Got here");

    $.ajax({
        type: "POST",
        url: "/api/v1/createRobot",
        data: formData,
        dataType: "json",
        contentType: "application/json",
        success: function (res) {
            console.log('Robot created successfully');
            if (res === 200) {
                // robot not null
                setToast("success", "", "Successfully created robot", true);
                window.location.href = `/`;
            }
        },
        error: function (xhr, status, error) {
            // error handling
            console.error('Error creating robot:', error);
            setToast("error", "Something went wrong", "Could not create robot");
        }
    })

});